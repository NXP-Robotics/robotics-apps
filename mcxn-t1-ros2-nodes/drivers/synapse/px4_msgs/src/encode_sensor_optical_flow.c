/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <ucdr/microcdr.h>

#include <px4_msgs_c/sensor_optical_flow.h>

static int write_cdr_header(struct ucdrBuffer *ub)
{
	const uint8_t hdr[4] = {0x00U, 0x01U, 0x00U, 0x00U};

	ucdr_serialize_array_uint8_t(ub, hdr, sizeof(hdr));
	
	ub->offset = 0;      /* internal offset for alignment restarts at 0 */
	ub->iterator = ub->init + 4; /* keep write pointer past the header    */
	ub->last_data_size = 0;
	return ub->error ? -1 : 0;
}

int px4_msgs_c_encode_sensor_optical_flow(struct ucdrBuffer *ub,
					  const px4_msgs_c_SensorOpticalFlow_t *m)
{
	if (write_cdr_header(ub) != 0) {
		return -1;
	}

	ucdr_serialize_uint64_t(ub, m->timestamp);
	ucdr_serialize_uint64_t(ub, m->timestamp_sample);
	ucdr_serialize_uint32_t(ub, m->device_id);
	ucdr_serialize_array_float(ub, m->pixel_flow, 2);
	ucdr_serialize_array_float(ub, m->delta_angle, 3);
	ucdr_serialize_bool(ub, m->delta_angle_available);
	ucdr_serialize_float(ub, m->distance_m);
	ucdr_serialize_bool(ub, m->distance_available);
	ucdr_serialize_uint32_t(ub, m->integration_timespan_us);
	ucdr_serialize_uint8_t(ub, m->quality);
	ucdr_serialize_uint32_t(ub, m->error_count);
	ucdr_serialize_float(ub, m->max_flow_rate);
	ucdr_serialize_float(ub, m->min_ground_distance);
	ucdr_serialize_float(ub, m->max_ground_distance);
	ucdr_serialize_uint8_t(ub, m->mode);

	return ub->error ? -1 : 0;
}
