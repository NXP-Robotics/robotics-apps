/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_SENSOR_GPS_H
#define PX4_MSGS_C_SENSOR_GPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* FIX_TYPE_* (u8); value 0 is also valid and means "no fix" even though it
 * has no named constant upstream. */
#define PX4_MSGS_C_GPS_FIX_TYPE_NONE          1
#define PX4_MSGS_C_GPS_FIX_TYPE_2D            2
#define PX4_MSGS_C_GPS_FIX_TYPE_3D            3
#define PX4_MSGS_C_GPS_FIX_TYPE_RTCM_DGNSS    4
#define PX4_MSGS_C_GPS_FIX_TYPE_RTK_FLOAT     5
#define PX4_MSGS_C_GPS_FIX_TYPE_RTK_FIXED     6
#define PX4_MSGS_C_GPS_FIX_TYPE_EXTRAPOLATED  8

/* JAMMING_STATE_* (u8) */
#define PX4_MSGS_C_JAMMING_STATE_UNKNOWN   0
#define PX4_MSGS_C_JAMMING_STATE_OK        1
#define PX4_MSGS_C_JAMMING_STATE_MITIGATED 2
#define PX4_MSGS_C_JAMMING_STATE_DETECTED  3

/* SPOOFING_STATE_* (u8) */
#define PX4_MSGS_C_SPOOFING_STATE_UNKNOWN   0
#define PX4_MSGS_C_SPOOFING_STATE_OK        1
#define PX4_MSGS_C_SPOOFING_STATE_MITIGATED 2
#define PX4_MSGS_C_SPOOFING_STATE_DETECTED  3

/* AUTHENTICATION_STATE_* (u8) */
#define PX4_MSGS_C_AUTHENTICATION_STATE_UNKNOWN      0
#define PX4_MSGS_C_AUTHENTICATION_STATE_INITIALIZING 1
#define PX4_MSGS_C_AUTHENTICATION_STATE_ERROR        2
#define PX4_MSGS_C_AUTHENTICATION_STATE_OK           3
#define PX4_MSGS_C_AUTHENTICATION_STATE_DISABLED     4

/* SYSTEM_ERROR_* (u32 bitmask) */
#define PX4_MSGS_C_SYSTEM_ERROR_OK 0U

/* RTCM_MSG_USED_* (u8) */
#define PX4_MSGS_C_RTCM_MSG_USED_UNKNOWN  0
#define PX4_MSGS_C_RTCM_MSG_USED_NOT_USED 1
#define PX4_MSGS_C_RTCM_MSG_USED_USED     2

struct px4_msgs_c_SensorGps {
	uint64_t timestamp;                     /* uint64 timestamp */
	uint64_t timestamp_sample;              /* uint64 timestamp_sample */
	uint32_t device_id;                     /* uint32 device_id */
	double   latitude_deg;                  /* float64 latitude_deg */
	double   longitude_deg;                 /* float64 longitude_deg */
	double   altitude_msl_m;                /* float64 altitude_msl_m */
	double   altitude_ellipsoid_m;          /* float64 altitude_ellipsoid_m */
	float    s_variance_m_s;                /* float32 s_variance_m_s */
	float    c_variance_rad;                /* float32 c_variance_rad */
	uint8_t  fix_type;                      /* uint8 fix_type */
	float    eph;                           /* float32 eph */
	float    epv;                           /* float32 epv */
	float    hdop;                          /* float32 hdop */
	float    vdop;                          /* float32 vdop */
	int32_t  noise_per_ms;                  /* int32 noise_per_ms */
	uint16_t automatic_gain_control;        /* uint16 automatic_gain_control */
	uint8_t  jamming_state;                 /* uint8 jamming_state */
	int32_t  jamming_indicator;             /* int32 jamming_indicator */
	uint8_t  spoofing_state;                /* uint8 spoofing_state */
	uint8_t  authentication_state;          /* uint8 authentication_state */
	float    vel_m_s;                       /* float32 vel_m_s */
	float    vel_n_m_s;                     /* float32 vel_n_m_s */
	float    vel_e_m_s;                     /* float32 vel_e_m_s */
	float    vel_d_m_s;                     /* float32 vel_d_m_s */
	float    cog_rad;                       /* float32 cog_rad */
	bool     vel_ned_valid;                 /* bool vel_ned_valid */
	int32_t  timestamp_time_relative;       /* int32 timestamp_time_relative */
	uint64_t time_utc_usec;                 /* uint64 time_utc_usec */
	uint8_t  satellites_used;               /* uint8 satellites_used */
	uint32_t system_error;                  /* uint32 system_error */
	float    heading;                       /* float32 heading */
	float    heading_offset;                /* float32 heading_offset */
	float    heading_accuracy;              /* float32 heading_accuracy */
	float    rtcm_injection_rate;           /* float32 rtcm_injection_rate */
	uint8_t  selected_rtcm_instance;        /* uint8 selected_rtcm_instance */
	bool     rtcm_crc_failed;               /* bool rtcm_crc_failed */
	uint8_t  rtcm_msg_used;                 /* uint8 rtcm_msg_used */
	float    antenna_offset_x;              /* float32 antenna_offset_x (FRD, m) */
	float    antenna_offset_y;              /* float32 antenna_offset_y */
	float    antenna_offset_z;              /* float32 antenna_offset_z */
};
typedef struct px4_msgs_c_SensorGps px4_msgs_c_SensorGps_t;

#define PX4_MSGS_C_SENSOR_GPS_CDR_MAX 208U

struct ucdrBuffer;
int px4_msgs_c_encode_sensor_gps(struct ucdrBuffer *ub,
				 const px4_msgs_c_SensorGps_t *m);

#endif /* PX4_MSGS_C_SENSOR_GPS_H */
