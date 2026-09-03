/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_SENSOR_MAG_H
#define PX4_MSGS_C_SENSOR_MAG_H

#include <stddef.h>
#include <stdint.h>

struct px4_msgs_c_SensorMag {
	uint64_t timestamp;         /* uint64 timestamp */
	uint64_t timestamp_sample;  /* uint64 timestamp_sample */
	uint32_t device_id;         /* uint32 device_id */
	float    x;                 /* float32 x (Gauss, FRD) */
	float    y;                 /* float32 y */
	float    z;                 /* float32 z */
	float    temperature;       /* float32 temperature */
	uint32_t error_count;       /* uint32 error_count */
};
typedef struct px4_msgs_c_SensorMag px4_msgs_c_SensorMag_t;

#define PX4_MSGS_C_SENSOR_MAG_CDR_MAX 64U

struct ucdrBuffer;
int px4_msgs_c_encode_sensor_mag(struct ucdrBuffer *ub,
				 const px4_msgs_c_SensorMag_t *m);

#endif /* PX4_MSGS_C_SENSOR_MAG_H */
