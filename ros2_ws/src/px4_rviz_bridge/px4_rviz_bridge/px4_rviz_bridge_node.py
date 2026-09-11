#!/usr/bin/env python3
"""px4_rviz_bridge

Converts PX4 sensor topics into RViz-friendly messages and drives a board pose:

  /fmu/in/distance_sensor      (px4_msgs/DistanceSensor)     -> /viz/range  (sensor_msgs/Range)
  /fmu/in/sensor_optical_flow  (px4_msgs/SensorOpticalFlow)  -> /viz/flow   (visualization_msgs/Marker ARROW)
  /fmu/in/sensor_combined      (px4_msgs/SensorCombined)     -> dynamic TF map->base_link (+ /viz/board CUBE)

Frames:
    map            world/fixed frame (RViz Fixed Frame)
    base_link      vehicle body (this node broadcasts map->base_link DYNAMICALLY
                   from the IMU attitude; the board CUBE lives here)
    flow_sensor    downward-facing sensor mount (STATIC child of base_link,
                   pitched +90 deg by the launch file; cone + arrow live here)

Attitude source:
    SensorCombined carries accel + gyro in FRD (NO orientation). This node runs
    a complementary filter: accelerometer gives drift-free roll/pitch (gravity),
    gyro gives smooth high-rate integration. There is NO magnetometer on the T1
    board, so YAW is gyro-integrated and WILL DRIFT. That is expected and fine
    for a movement demo. We convert FRD->FLU for RViz (negate y,z).

QoS note:
    PX4 / the zenoh bridge publish BEST_EFFORT. A default (RELIABLE) subscriber
    matches nothing even though `ros2 topic echo` works. We subscribe BEST_EFFORT.
    The /viz/* publishers and the TF are our own (RViz sees them with defaults).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import DistanceSensor, SensorOpticalFlow, SensorCombined
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster


# PX4 SensorOrientation enum values we care about (see px4 sensor_orientation.h)
ROTATION_DOWNWARD_FACING = 25


class Px4RvizBridge(Node):
    def __init__(self):
        super().__init__('px4_rviz_bridge')

        # ---- parameters (override on the CLI or in the launch file) ----
        self.declare_parameter('sensor_frame', 'flow_sensor')
        self.declare_parameter('range_topic_in', '/fmu/in/distance_sensor')
        self.declare_parameter('flow_topic_in', '/fmu/in/sensor_optical_flow')
        # optical flow arrow visual scale (pixel_flow is small angular rate in rad;
        # multiply so the arrow is visible in RViz). Purely cosmetic.
        self.declare_parameter('flow_arrow_scale', 5.0)
        # default field-of-view (rad) used for the Range cone when the message
        # does not carry one (your h_fov/v_fov come through as 0.0).
        self.declare_parameter('default_fov_rad', 0.5)
        # ---- IMU / board params ----
        self.declare_parameter('imu_topic_in', '/fmu/in/sensor_combined')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # complementary-filter weight on the gyro (0..1). 0.98 = trust gyro at
        # high frequency, let accel correct roll/pitch slowly. Yaw is gyro-only.
        self.declare_parameter('comp_alpha', 0.98)
        # board CUBE dimensions in metres (x=length, y=width, z=thickness).
        self.declare_parameter('board_size', [0.08, 0.05, 0.015])

        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.flow_arrow_scale = float(self.get_parameter('flow_arrow_scale').value)
        self.default_fov = float(self.get_parameter('default_fov_rad').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.comp_alpha = float(self.get_parameter('comp_alpha').value)
        self.board_size = [float(v) for v in self.get_parameter('board_size').value]

        # complementary-filter state (radians)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._imu_last_us = 0  # for dt if the message dt is unusable

        # ---- BEST_EFFORT QoS to match PX4 publishers ----
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---- subscriptions ----
        self.create_subscription(
            DistanceSensor,
            self.get_parameter('range_topic_in').value,
            self.on_distance, sensor_qos)
        self.create_subscription(
            SensorOpticalFlow,
            self.get_parameter('flow_topic_in').value,
            self.on_flow, sensor_qos)
        self.create_subscription(
            SensorCombined,
            self.get_parameter('imu_topic_in').value,
            self.on_imu, sensor_qos)

        # ---- publishers (RELIABLE; RViz default subscribes fine) ----
        self.range_pub = self.create_publisher(Range, '/viz/range', 10)
        self.flow_pub = self.create_publisher(Marker, '/viz/flow', 10)
        self.board_pub = self.create_publisher(Marker, '/viz/board', 10)

        # ---- dynamic TF broadcaster for map->base_link (driven by IMU) ----
        self.tf_broadcaster = TransformBroadcaster(self)
        # publish an identity map->base_link + a board at startup so the scene
        # is populated even before the first IMU sample arrives.
        self._publish_board()
        self._broadcast_base(0.0, 0.0, 0.0)

        # re-publish the board marker periodically (markers are not latched and
        # a late-joining RViz may miss the one-shot publish).
        self.create_timer(1.0, self._publish_board)

        self.get_logger().info(
            'px4_rviz_bridge up. BEST_EFFORT subs: '
            f'{self.get_parameter("range_topic_in").value}, '
            f'{self.get_parameter("flow_topic_in").value}, '
            f'{self.get_parameter("imu_topic_in").value}. '
            f'Publishing /viz/range, /viz/flow, /viz/board and dynamic '
            f'{self.map_frame}->{self.base_frame} TF. '
            'NOTE: no magnetometer on board -> yaw drifts (expected).')

    # ------------------------------------------------------------------ #
    def on_distance(self, msg: DistanceSensor):
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = self.sensor_frame

        # Range.radiation_type: 0=ULTRASOUND, 1=INFRARED. PX4 type enum differs,
        # so just pick INFRARED for a laser/ToF-style rangefinder cone.
        r.radiation_type = Range.INFRARED

        # h_fov/v_fov arrive as 0.0 in your data -> fall back to a sane default.
        fov = msg.h_fov if msg.h_fov > 0.0 else self.default_fov
        r.field_of_view = float(fov)

        r.min_range = float(msg.min_distance)
        r.max_range = float(msg.max_distance)
        # clamp: RViz ignores/greys negative or out-of-band ranges.
        r.range = float(max(msg.current_distance, 0.0))

        self.range_pub.publish(r)

    # ------------------------------------------------------------------ #
    def on_flow(self, msg: SensorOpticalFlow):
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = self.sensor_frame
        mk.ns = 'optical_flow'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD

        # Arrow defined by two points: tail at origin, head along the flow vector.
        # pixel_flow[0]=x (rad), pixel_flow[1]=y (rad) angular flow. We draw it in
        # the sensor XY plane, scaled for visibility. This is a synthetic visual,
        # not a physical position.
        fx = float(msg.pixel_flow[0]) * self.flow_arrow_scale
        fy = float(msg.pixel_flow[1]) * self.flow_arrow_scale
        mk.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=fx, y=fy, z=0.0),
        ]

        # ARROW with points uses scale.x=shaft dia, scale.y=head dia, scale.z=head len
        mk.scale.x = 0.02
        mk.scale.y = 0.04
        mk.scale.z = 0.06

        # colour by quality: red (low) -> green (high). quality is 0..255.
        q = max(0.0, min(1.0, msg.quality / 255.0))
        mk.color.r = float(1.0 - q)
        mk.color.g = float(q)
        mk.color.b = 0.0
        mk.color.a = 1.0

        # keep it around ~0.5s so a momentarily-zero flow doesn't flicker off
        mk.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        self.flow_pub.publish(mk)

    # ------------------------------------------------------------------ #
    def on_imu(self, msg: SensorCombined):
        """Complementary filter on SensorCombined (FRD) -> board attitude.

        gyro_rad and accelerometer_m_s2 are FRD. We integrate gyro for the
        high-frequency attitude and correct roll/pitch with the accelerometer
        gravity direction. Yaw has no absolute reference (no mag) so it is pure
        gyro integration and drifts. Result is converted FRD->FLU for RViz.
        """
        # integration period: prefer the message's gyro_integral_dt (us); fall
        # back to wall-clock delta if it is zero/unset.
        dt = msg.gyro_integral_dt * 1e-6 if msg.gyro_integral_dt > 0 else 0.0
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01  # sane default (~100 Hz) if the field is unusable

        gx, gy, gz = (float(msg.gyro_rad[0]), float(msg.gyro_rad[1]),
                      float(msg.gyro_rad[2]))
        ax, ay, az = (float(msg.accelerometer_m_s2[0]),
                      float(msg.accelerometer_m_s2[1]),
                      float(msg.accelerometer_m_s2[2]))

        # --- gyro integration (FRD body rates) ---
        roll_g = self._roll + gx * dt
        pitch_g = self._pitch + gy * dt
        yaw_g = self._yaw + gz * dt

        # --- accelerometer roll/pitch (drift-free, from gravity) ---
        # FRD: at rest az ~ +9.81 (gravity points down = +z). Standard formulas.
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if 5.0 < acc_norm < 15.0:  # only trust when near 1 g (not accelerating hard)
            roll_a = math.atan2(ay, az)
            pitch_a = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            a = self.comp_alpha
            self._roll = a * roll_g + (1.0 - a) * roll_a
            self._pitch = a * pitch_g + (1.0 - a) * pitch_a
        else:
            # under strong linear acceleration, trust gyro only for this step
            self._roll = roll_g
            self._pitch = pitch_g
        self._yaw = yaw_g  # no absolute yaw reference -> integrate (drifts)

        # FRD attitude -> FLU (RViz/ENU-style body): negate pitch and yaw.
        # (roll about forward x stays; y and z axes flip sign FRD->FLU.)
        self._broadcast_base(self._roll, -self._pitch, -self._yaw)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def _broadcast_base(self, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        x, y, z, w = self._rpy_to_quat(roll, pitch, yaw)
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(t)

    def _publish_board(self):
        """Thin CUBE marker representing the PCB, anchored at base_link origin."""
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = self.base_frame
        mk.ns = 'board'
        mk.id = 0
        mk.type = Marker.CUBE
        mk.action = Marker.ADD
        mk.pose.position.x = 0.0
        mk.pose.position.y = 0.0
        mk.pose.position.z = 0.0
        mk.pose.orientation.w = 1.0
        mk.scale.x = self.board_size[0]
        mk.scale.y = self.board_size[1]
        mk.scale.z = self.board_size[2]
        mk.color.r = 0.10
        mk.color.g = 0.35
        mk.color.b = 0.80
        mk.color.a = 0.9
        self.board_pub.publish(mk)


def main(args=None):
    rclpy.init(args=args)
    node = Px4RvizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
