/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_SENSOR_OPTICAL_FLOW_H
#define PX4_MSGS_C_SENSOR_OPTICAL_FLOW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Mode enum, MODE_* constants from the .msg (u8). */
#define PX4_MSGS_C_OPTICAL_FLOW_MODE_UNKNOWN        0
#define PX4_MSGS_C_OPTICAL_FLOW_MODE_BRIGHT         1
#define PX4_MSGS_C_OPTICAL_FLOW_MODE_LOWLIGHT       2
#define PX4_MSGS_C_OPTICAL_FLOW_MODE_SUPER_LOWLIGHT 3

struct px4_msgs_c_SensorOpticalFlow {
	uint64_t timestamp;                    /* .msg: uint64 timestamp */
	uint64_t timestamp_sample;             /* .msg: uint64 timestamp_sample */
	uint32_t device_id;                    /* .msg: uint32 device_id */
	float    pixel_flow[2];                /* .msg: float32[2] pixel_flow */
	float    delta_angle[3];               /* .msg: float32[3] delta_angle */
	bool     delta_angle_available;        /* .msg: bool delta_angle_available */
	float    distance_m;                   /* .msg: float32 distance_m */
	bool     distance_available;           /* .msg: bool distance_available */
	uint32_t integration_timespan_us;      /* .msg: uint32 integration_timespan_us */
	uint8_t  quality;                      /* .msg: uint8 quality */
	uint32_t error_count;                  /* .msg: uint32 error_count */
	float    max_flow_rate;                /* .msg: float32 max_flow_rate */
	float    min_ground_distance;          /* .msg: float32 min_ground_distance */
	float    max_ground_distance;          /* .msg: float32 max_ground_distance */
	uint8_t  mode;                         /* .msg: uint8 mode */
};
typedef struct px4_msgs_c_SensorOpticalFlow px4_msgs_c_SensorOpticalFlow_t;

/* Upper bound on CDR body + 4-byte encapsulation header (worst case with
 * alignment padding); used to size scratch buffers. */
#define PX4_MSGS_C_SENSOR_OPTICAL_FLOW_CDR_MAX 96U

struct ucdrBuffer;
int px4_msgs_c_encode_sensor_optical_flow(struct ucdrBuffer *ub,
					  const px4_msgs_c_SensorOpticalFlow_t *m);

#endif /* PX4_MSGS_C_SENSOR_OPTICAL_FLOW_H */
