/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_DISTANCE_SENSOR_H
#define PX4_MSGS_C_DISTANCE_SENSOR_H

#include <stddef.h>
#include <stdint.h>

/* MAV_DISTANCE_SENSOR_* (u8) */
#define PX4_MSGS_C_DISTANCE_SENSOR_TYPE_LASER      0
#define PX4_MSGS_C_DISTANCE_SENSOR_TYPE_ULTRASOUND 1
#define PX4_MSGS_C_DISTANCE_SENSOR_TYPE_INFRARED   2
#define PX4_MSGS_C_DISTANCE_SENSOR_TYPE_RADAR      3

/* ROTATION_* (u8); only the one this bridge uses is named here. */
#define PX4_MSGS_C_ROTATION_DOWNWARD_FACING 25

/* mode enum, MODE_* constants from the .msg (u8). */
#define PX4_MSGS_C_DISTANCE_SENSOR_MODE_UNKNOWN  0
#define PX4_MSGS_C_DISTANCE_SENSOR_MODE_ENABLED  1
#define PX4_MSGS_C_DISTANCE_SENSOR_MODE_DISABLED 2

struct px4_msgs_c_DistanceSensor {
	uint64_t timestamp;         /* .msg: uint64 timestamp */
	uint32_t device_id;         /* .msg: uint32 device_id */
	float    min_distance;      /* .msg: float32 min_distance */
	float    max_distance;      /* .msg: float32 max_distance */
	float    current_distance;  /* .msg: float32 current_distance */
	float    variance;          /* .msg: float32 variance */
	int8_t   signal_quality;    /* .msg: int8 signal_quality (0..100, -1 = unknown) */
	uint8_t  type;              /* .msg: uint8 type */
	float    h_fov;             /* .msg: float32 h_fov (radians) */
	float    v_fov;             /* .msg: float32 v_fov (radians) */
	float    q[4];              /* .msg: float32[4] q */
	uint8_t  orientation;       /* .msg: uint8 orientation */
	uint8_t  mode;              /* .msg: uint8 mode (last field) */
};

typedef struct px4_msgs_c_DistanceSensor px4_msgs_c_DistanceSensor_t;

#define PX4_MSGS_C_DISTANCE_SENSOR_CDR_MAX 96U

struct ucdrBuffer;
int px4_msgs_c_encode_distance_sensor(struct ucdrBuffer *ub,
				      const px4_msgs_c_DistanceSensor_t *m);

#endif /* PX4_MSGS_C_DISTANCE_SENSOR_H */
