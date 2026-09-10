/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ucdr/microcdr.h>

#include <px4_msgs_c/sensor_combined.h>

static int write_cdr_header(struct ucdrBuffer *ub)
{
	const uint8_t hdr[4] = {0x00U, 0x01U, 0x00U, 0x00U};

	ucdr_serialize_array_uint8_t(ub, hdr, sizeof(hdr));
	ub->offset = 0;
	ub->iterator = ub->init + 4;
	ub->last_data_size = 0;
	return ub->error ? -1 : 0;
}

/*
 * CDR body in SensorCombined.msg IDL field order (constants excluded).
 * micro-CDR applies natural alignment per primitive, so we just serialize in
 * declaration order.
 */
int px4_msgs_c_encode_sensor_combined(struct ucdrBuffer *ub,
				      const px4_msgs_c_SensorCombined_t *m)
{
	if (write_cdr_header(ub) != 0) {
		return -1;
	}

	ucdr_serialize_uint64_t(ub, m->timestamp);
	ucdr_serialize_array_float(ub, m->gyro_rad, 3);
	ucdr_serialize_uint32_t(ub, m->gyro_integral_dt);
	ucdr_serialize_int32_t(ub, m->accelerometer_timestamp_relative);
	ucdr_serialize_array_float(ub, m->accelerometer_m_s2, 3);
	ucdr_serialize_uint32_t(ub, m->accelerometer_integral_dt);
	ucdr_serialize_uint8_t(ub, m->accelerometer_clipping);
	ucdr_serialize_uint8_t(ub, m->gyro_clipping);
	ucdr_serialize_uint8_t(ub, m->accel_calibration_count);
	ucdr_serialize_uint8_t(ub, m->gyro_calibration_count);

	return ub->error ? -1 : 0;
}
