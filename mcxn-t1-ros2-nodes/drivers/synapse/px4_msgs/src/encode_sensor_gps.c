/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ucdr/microcdr.h>

#include <px4_msgs_c/sensor_gps.h>

static int write_cdr_header(struct ucdrBuffer *ub)
{
	const uint8_t hdr[4] = {0x00U, 0x01U, 0x00U, 0x00U};

	ucdr_serialize_array_uint8_t(ub, hdr, sizeof(hdr));
	ub->offset = 0;
	ub->iterator = ub->init + 4;
	ub->last_data_size = 0;
	return ub->error ? -1 : 0;
}

int px4_msgs_c_encode_sensor_gps(struct ucdrBuffer *ub,
				 const px4_msgs_c_SensorGps_t *m)
{
	if (write_cdr_header(ub) != 0) {
		return -1;
	}

	ucdr_serialize_uint64_t(ub, m->timestamp);
	ucdr_serialize_uint64_t(ub, m->timestamp_sample);
	ucdr_serialize_uint32_t(ub, m->device_id);
	ucdr_serialize_double(ub, m->latitude_deg);
	ucdr_serialize_double(ub, m->longitude_deg);
	ucdr_serialize_double(ub, m->altitude_msl_m);
	ucdr_serialize_double(ub, m->altitude_ellipsoid_m);
	ucdr_serialize_float(ub, m->s_variance_m_s);
	ucdr_serialize_float(ub, m->c_variance_rad);
	ucdr_serialize_uint8_t(ub, m->fix_type);
	ucdr_serialize_float(ub, m->eph);
	ucdr_serialize_float(ub, m->epv);
	ucdr_serialize_float(ub, m->hdop);
	ucdr_serialize_float(ub, m->vdop);
	ucdr_serialize_int32_t(ub, m->noise_per_ms);
	ucdr_serialize_uint16_t(ub, m->automatic_gain_control);
	ucdr_serialize_uint8_t(ub, m->jamming_state);
	ucdr_serialize_int32_t(ub, m->jamming_indicator);
	ucdr_serialize_uint8_t(ub, m->spoofing_state);
	ucdr_serialize_uint8_t(ub, m->authentication_state);
	ucdr_serialize_float(ub, m->vel_m_s);
	ucdr_serialize_float(ub, m->vel_n_m_s);
	ucdr_serialize_float(ub, m->vel_e_m_s);
	ucdr_serialize_float(ub, m->vel_d_m_s);
	ucdr_serialize_float(ub, m->cog_rad);
	ucdr_serialize_bool(ub, m->vel_ned_valid);
	ucdr_serialize_int32_t(ub, m->timestamp_time_relative);
	ucdr_serialize_uint64_t(ub, m->time_utc_usec);
	ucdr_serialize_uint8_t(ub, m->satellites_used);
	ucdr_serialize_uint32_t(ub, m->system_error);
	ucdr_serialize_float(ub, m->heading);
	ucdr_serialize_float(ub, m->heading_offset);
	ucdr_serialize_float(ub, m->heading_accuracy);
	ucdr_serialize_float(ub, m->rtcm_injection_rate);
	ucdr_serialize_uint8_t(ub, m->selected_rtcm_instance);
	ucdr_serialize_bool(ub, m->rtcm_crc_failed);
	ucdr_serialize_uint8_t(ub, m->rtcm_msg_used);
	ucdr_serialize_float(ub, m->antenna_offset_x);
	ucdr_serialize_float(ub, m->antenna_offset_y);
	ucdr_serialize_float(ub, m->antenna_offset_z);

	return ub->error ? -1 : 0;
}
