#!/usr/bin/env python3
"""Launch the px4_rviz_bridge converter, static TF tree, and RViz.

TF tree:
    map --> base_link --> flow_sensor

  map -> base_link:   identity (vehicle sitting at origin). Replace with a real
                      odometry source later if you fuse pose.
  base_link -> flow_sensor: rotate so the sensor's +X axis points DOWN, matching
                      PX4 orientation 25 (ROTATION_DOWNWARD_FACING). A pitch of
                      +90 deg about Y turns body-forward (+X) into body-down.
                      This makes the sensor_msgs/Range cone project downward in
                      RViz, which is what a downward rangefinder/optical-flow
                      module should look like.

static_transform_publisher arg order:  x y z  yaw pitch roll  parent child
"""

import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_rviz_bridge')
    default_rviz = os.path.join(pkg_share, 'rviz', 'px4_flow.rviz')

    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    half_pi = str(math.pi / 2.0)

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz with the bundled config'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='Path to an .rviz config file'),

        # ---- converter node ----
        Node(
            package='px4_rviz_bridge',
            executable='px4_rviz_bridge',
            name='px4_rviz_bridge',
            output='screen',
            parameters=[{
                'sensor_frame': 'flow_sensor',
                'flow_arrow_scale': 5.0,
                'default_fov_rad': 0.5,
                'comp_alpha': 0.98,
                'board_size': [0.08, 0.05, 0.015],
            }],
        ),

        # NOTE: map -> base_link is NO LONGER static. The px4_rviz_bridge node
        # broadcasts it DYNAMICALLY from the IMU (SensorCombined) so the board
        # rotates. Publishing a static one here too would fight the node's TF.

        # ---- static TF: base_link -> flow_sensor (pitch +90deg = point +X down) ----
        # args: x y z yaw pitch roll parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_flow',
            arguments=['0', '0', '0', '0', half_pi, '0', 'base_link', 'flow_sensor'],
        ),

        # ---- RViz ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
