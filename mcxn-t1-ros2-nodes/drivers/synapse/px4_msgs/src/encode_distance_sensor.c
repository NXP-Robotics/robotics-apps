/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ucdr/microcdr.h>

#include <px4_msgs_c/distance_sensor.h>

static int write_cdr_header(struct ucdrBuffer *ub)
{
	const uint8_t hdr[4] = {0x00U, 0x01U, 0x00U, 0x00U};

	ucdr_serialize_array_uint8_t(ub, hdr, sizeof(hdr));
	ub->offset = 0;
	ub->iterator = ub->init + 4;
	ub->last_data_size = 0;
	return ub->error ? -1 : 0;
}

int px4_msgs_c_encode_distance_sensor(struct ucdrBuffer *ub,
				      const px4_msgs_c_DistanceSensor_t *m)
{
	if (write_cdr_header(ub) != 0) {
		return -1;
	}

	ucdr_serialize_uint64_t(ub, m->timestamp);
	ucdr_serialize_uint32_t(ub, m->device_id);
	ucdr_serialize_float(ub, m->min_distance);
	ucdr_serialize_float(ub, m->max_distance);
	ucdr_serialize_float(ub, m->current_distance);
	ucdr_serialize_float(ub, m->variance);
	ucdr_serialize_int8_t(ub, m->signal_quality);
	ucdr_serialize_uint8_t(ub, m->type);
	ucdr_serialize_float(ub, m->h_fov);
	ucdr_serialize_float(ub, m->v_fov);
	ucdr_serialize_array_float(ub, m->q, 4);
	ucdr_serialize_uint8_t(ub, m->orientation);
	ucdr_serialize_uint8_t(ub, m->mode);

	return ub->error ? -1 : 0;
}

