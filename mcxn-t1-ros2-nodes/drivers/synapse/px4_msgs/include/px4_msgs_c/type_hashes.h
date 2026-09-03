/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_TYPE_HASHES_H
#define PX4_MSGS_C_TYPE_HASHES_H

#define PX4_MSGS_C_ROS_TYPE_SENSOR_OPTICAL_FLOW "px4_msgs::msg::dds_::SensorOpticalFlow_"
#define PX4_MSGS_C_ROS_TYPE_DISTANCE_SENSOR     "px4_msgs::msg::dds_::DistanceSensor_"
#define PX4_MSGS_C_ROS_TYPE_SENSOR_GPS          "px4_msgs::msg::dds_::SensorGps_"
#define PX4_MSGS_C_ROS_TYPE_SENSOR_MAG          "px4_msgs::msg::dds_::SensorMag_"

#define PX4_MSGS_C_TOPIC_SENSOR_OPTICAL_FLOW "sensor_optical_flow"
#define PX4_MSGS_C_TOPIC_DISTANCE_SENSOR     "distance_sensor"
#define PX4_MSGS_C_TOPIC_SENSOR_GPS          "sensor_gps"
#define PX4_MSGS_C_TOPIC_SENSOR_MAG          "sensor_mag"

/* Real RIHS01 hashes captured on Jazzy against the pinned px4_msgs commit. */
#define PX4_MSGS_C_RIHS_SENSOR_OPTICAL_FLOW                                                        \
	"RIHS01_25177fb9630317cd454c4f4e0281d8585eca7843fb74b23c978b73569f3a3a57"
#define PX4_MSGS_C_RIHS_DISTANCE_SENSOR                                                            \
	"RIHS01_a8457d649dbcc726dce2558cf8a5732dc0b74d518bdb5e89248c0c515e8f80f5"
#define PX4_MSGS_C_RIHS_SENSOR_GPS                                                                 \
	"RIHS01_fbef1acd7734fab29377906ae90f6493e9a07724a6a21997fb8fd51d6771b958"
#define PX4_MSGS_C_RIHS_SENSOR_MAG                                                                 \
	"RIHS01_217da8c1190f1ac948c3a5abe4e7489412655fc24f2dc93c51fa5087e5140d89"

#endif /* PX4_MSGS_C_TYPE_HASHES_H */
