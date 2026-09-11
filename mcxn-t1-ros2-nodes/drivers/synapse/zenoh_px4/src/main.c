/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_PERSIST)
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#endif

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <zenoh-pico.h>

#include <ucdr/microcdr.h>

#include <synapse_topic_list.h>

#include <synapse_pb/argus_results.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/pixart_paa3905.pb.h>

#include <px4_msgs_c/distance_sensor.h>
#include <px4_msgs_c/sensor_combined.h>
#include <px4_msgs_c/sensor_gps.h>
#include <px4_msgs_c/sensor_mag.h>
#include <px4_msgs_c/sensor_optical_flow.h>
#include <px4_msgs_c/type_hashes.h>

LOG_MODULE_REGISTER(synapse_zenoh_px4, CONFIG_SYNAPSE_ZENOH_PX4_LOG_LEVEL);


#define MY_STACK_SIZE 8192
#define MY_PRIORITY   4
#define KEYEXPR_MAX   200
#define CDR_BUF_MAX   256U
#define POLL_MS       10
#define DIST_MIN_M 0.08f
#define DIST_MAX_M 30.0f

/* Consecutive publisher_put failures tolerated before the session is
 * considered dead and an automatic reconnect is attempted. This matters in
 * client mode: if the TCP link to the router drops, zenoh-pico tears down the
 * transport and every subsequent put fails until the session is reopened.
 *
 * With BEST_EFFORT + DROP congestion control (see declare_publishers) a put no
 * longer fails on mere back-pressure, so a high count now genuinely indicates a
 * dead transport rather than a busy one. Keep it generous to avoid tearing down
 * a live session over a transient blip.
 */
#define PUT_FAIL_LIMIT 60

/* Minimum time between reconnect attempts (ms). Without this the loop can
 * reconnect many times per second, which never lets the far side (rmw_zenoh /
 * RViz) re-discover the liveliness graph -> RViz freezes and never recovers.
 * A cooldown gives the transport time to actually re-establish and be used. */
#define RECONNECT_COOLDOWN_MS 3000

/* Optical-flow producer metadata (PAA3905 / AFBR-S50). These are sensor
 * datasheet constants used by EKF2 for gating; confirm against the parts fitted.
 */
#define FLOW_MAX_RATE_RAD_S 7.4f       /* PAA3905-class angular-rate saturation */
#define FLOW_MIN_GND_DIST_M DIST_MIN_M /* AFBR-S50 min valid range */
#define FLOW_MAX_GND_DIST_M DIST_MAX_M /* AFBR-S50 max valid range */
/* Per-count angular resolution of the PAA3905 flow output, in radians.
 * pixel_flow (rad) = raw_delta_counts * FLOW_RAD_PER_COUNT. CALIBRATE this
 * against a known-motion test for your optics; the value below is a starting
 * point derived from CONFIG (micro-rad per count). */
#define FLOW_RAD_PER_COUNT (CONFIG_SYNAPSE_ZENOH_PX4_FLOW_URAD_PER_COUNT * 1.0e-6f)
/* Clamp for the stamp-derived integration window (guards huge/negative dt). */
#define FLOW_DT_MAX_US 100000ULL /* 100 ms */

/* CDR/ROS2 media type used on every publisher-put. */
#define CDR_ENCODING_STR "application/octet-stream;encoding=cdr"

#if (CONFIG_SYNAPSE_ZENOH_PX4_OPTICAL_FLOW_RATE_HZ != 0)
#define BRIDGE_TX_OPTICAL_FLOW 1
#else
#define BRIDGE_TX_OPTICAL_FLOW 0
#endif

#if (CONFIG_SYNAPSE_ZENOH_PX4_DISTANCE_SENSOR_RATE_HZ != 0)
#define BRIDGE_TX_DISTANCE_SENSOR 1
#else
#define BRIDGE_TX_DISTANCE_SENSOR 0
#endif

#if (CONFIG_SYNAPSE_ZENOH_PX4_GNSS_RATE_HZ != 0)
#define BRIDGE_TX_SENSOR_GPS 1
#else
#define BRIDGE_TX_SENSOR_GPS 0
#endif

#if (CONFIG_SYNAPSE_ZENOH_PX4_MAG_RATE_HZ != 0)
#define BRIDGE_TX_SENSOR_MAG 1
#else
#define BRIDGE_TX_SENSOR_MAG 0
#endif

#if (CONFIG_SYNAPSE_ZENOH_PX4_SENSOR_COMBINED_RATE_HZ != 0)
#define BRIDGE_TX_SENSOR_COMBINED 1
#else
#define BRIDGE_TX_SENSOR_COMBINED 0
#endif

#define BRIDGE_TX_COUNT                                                                            \
	(BRIDGE_TX_OPTICAL_FLOW + BRIDGE_TX_DISTANCE_SENSOR + BRIDGE_TX_SENSOR_GPS +                \
	 BRIDGE_TX_SENSOR_MAG + BRIDGE_TX_SENSOR_COMBINED)

static K_THREAD_STACK_DEFINE(g_stack, MY_STACK_SIZE);

struct context {
	struct zros_node node;

#if BRIDGE_TX_OPTICAL_FLOW
	synapse_pb_PixartPAA3905        z_flow;
	px4_msgs_c_SensorOpticalFlow_t  p_flow;
	uint8_t                         cdr_flow[CDR_BUF_MAX];
#endif
#if BRIDGE_TX_DISTANCE_SENSOR
	synapse_pb_ArgusResults         z_dist_src; /* range comes from argus bin */
	px4_msgs_c_DistanceSensor_t     p_dist;
	uint8_t                         cdr_dist[CDR_BUF_MAX];
#endif
#if BRIDGE_TX_SENSOR_GPS
	synapse_pb_NavSatFix            z_gnss;
	px4_msgs_c_SensorGps_t          p_gps;
	uint8_t                         cdr_gps[CDR_BUF_MAX];
#endif
#if BRIDGE_TX_SENSOR_MAG
	synapse_pb_MagneticField        z_mag;
	px4_msgs_c_SensorMag_t          p_mag;
	uint8_t                         cdr_mag[CDR_BUF_MAX];
#endif
#if BRIDGE_TX_SENSOR_COMBINED
	synapse_pb_Imu                  z_imu;
	px4_msgs_c_SensorCombined_t     p_imu;
	uint8_t                         cdr_imu[CDR_BUF_MAX];
#endif

	z_owned_session_t sessions[1];

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	uint8_t          source_gid[16];
	char             zid_hex[33]; /* 32 hex chars + NUL */
	z_owned_liveliness_token_t node_token;
	bool rmw_declared;
#endif


	struct k_sem     running;
	struct k_thread  thread_data;
};

static struct context g_ctx = {
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
};

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG)

enum zpx4_mode {
	ZPX4_MODE_PEER = 0,   /* node LISTENs, PX4 connects */
	ZPX4_MODE_CLIENT = 1, /* node CONNECTs to a zenoh router */
};

#define ZPX4_LOCATOR_MAX 64

struct zpx4_run_cfg {
	char    locator[ZPX4_LOCATOR_MAX];
	uint8_t mode; /* enum zpx4_mode */
};

static struct zpx4_run_cfg g_run_cfg = {
	.locator = CONFIG_SYNAPSE_ZENOH_PX4_LOCATOR,
#if defined(CONFIG_SYNAPSE_ZENOH_PX4_MODE_PEER)
	.mode = ZPX4_MODE_PEER,
#else
	.mode = ZPX4_MODE_CLIENT,
#endif
};

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_PERSIST)

/* Persisted config: a single packed record (mode + locator) written under one
 * NVS id, so a save is one atomic write and a stale/short record is rejected.
 */
struct zpx4_persist {
	uint8_t mode;
	char    locator[ZPX4_LOCATOR_MAX];
} __packed;

/* NVS store on the board `storage_partition`, mirroring the kvss/nvs sample. */
#define ZPX4_NVS_PARTITION      storage_partition
#define ZPX4_NVS_DEVICE         PARTITION_DEVICE(ZPX4_NVS_PARTITION)
#define ZPX4_NVS_OFFSET         PARTITION_OFFSET(ZPX4_NVS_PARTITION)
#define ZPX4_NVS_SECTOR_COUNT   3U
#define ZPX4_CFG_ID             1     /* NVS entry id for the config record */

static struct nvs_fs g_nvs;
static bool          g_nvs_ready;

static bool zpx4_locator_ok(const char *loc)
{
	return strncmp(loc, "tcp/", 4) == 0 || strncmp(loc, "udp/", 4) == 0;
}

/* Mount the NVS file system on storage_partition. Returns 0 on success. */
static int zpx4_nvs_mount(void)
{
	struct flash_pages_info info;
	int rc;

	if (g_nvs_ready) {
		return 0;
	}

	g_nvs.flash_device = ZPX4_NVS_DEVICE;
	if (!device_is_ready(g_nvs.flash_device)) {
		LOG_WRN("persist: flash device not ready");
		return -ENODEV;
	}
	g_nvs.offset = ZPX4_NVS_OFFSET;
	rc = flash_get_page_info_by_offs(g_nvs.flash_device, g_nvs.offset, &info);
	if (rc) {
		LOG_WRN("persist: page info failed (%d)", rc);
		return rc;
	}
	g_nvs.sector_size = info.size;
	g_nvs.sector_count = ZPX4_NVS_SECTOR_COUNT;

	rc = nvs_mount(&g_nvs);
	if (rc) {
		LOG_WRN("persist: nvs_mount failed (%d)", rc);
		return rc;
	}
	g_nvs_ready = true;
	return 0;
}

/* Load persisted config (if any) into g_run_cfg. Runs before the bridge is
 * auto-started so the stored config becomes the power-on config. If nothing is
 * stored, or the record is invalid, g_run_cfg keeps its build-time defaults.
 */
static void zpx4_persist_load(void)
{
	struct zpx4_persist rec;
	ssize_t rc;

	if (zpx4_nvs_mount() != 0) {
		return;
	}

	rc = nvs_read(&g_nvs, ZPX4_CFG_ID, &rec, sizeof(rec));
	if (rc != (ssize_t)sizeof(rec)) {
		/* rc <= 0: no record -> keep defaults. rc != size: stale layout. */
		return;
	}
	if (rec.mode > ZPX4_MODE_CLIENT) {
		LOG_WRN("persist: bad stored mode, using defaults");
		return;
	}
	rec.locator[sizeof(rec.locator) - 1] = '\0';
	if (!zpx4_locator_ok(rec.locator)) {
		LOG_WRN("persist: invalid stored locator, using defaults");
		return;
	}

	g_run_cfg.mode = rec.mode;
	(void)snprintf(g_run_cfg.locator, sizeof(g_run_cfg.locator), "%s", rec.locator);
	LOG_INF("persist: loaded %s %s",
		g_run_cfg.mode == ZPX4_MODE_PEER ? "peer" : "client", g_run_cfg.locator);
}

#endif /* CONFIG_SYNAPSE_ZENOH_PX4_PERSIST */

#endif /* CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG */


/* -------- map functions: ZROS payload -> px4 struct -------- */

static inline uint64_t stamp_to_us(const synapse_pb_Timestamp *ts)
{
	int64_t sec = ts->seconds;
	int64_t nsec = ts->nanos;

	sec += nsec / 1000000000;
	nsec %= 1000000000;
	if (nsec < 0) {
		nsec += 1000000000;
		sec -= 1;
	}
	if (sec < 0) {
		return 0;
	}
	return (uint64_t)sec * 1000000ULL + (uint64_t)nsec / 1000ULL;
}


#if BRIDGE_TX_OPTICAL_FLOW

static uint8_t px4_flow_mode_from_pb(uint32_t pb_mode)
{
	switch (pb_mode) {
	case 0:
		return PX4_MSGS_C_OPTICAL_FLOW_MODE_BRIGHT;
	case 1:
		return PX4_MSGS_C_OPTICAL_FLOW_MODE_LOWLIGHT;
	case 2:
		return PX4_MSGS_C_OPTICAL_FLOW_MODE_SUPER_LOWLIGHT;
	default:
		return PX4_MSGS_C_OPTICAL_FLOW_MODE_UNKNOWN;
	}
}

/* Latest ground range (m), cached from the argus/distance path so it can be
 * embedded in each optical-flow sample. Written by map_distance_sensor(),
 * read by map_optical_flow(). Single writer/reader on the same bridge thread,
 * so no lock is needed. Defined once (outside the per-topic blocks) so it is
 * available whenever the distance path is compiled in. */
#if BRIDGE_TX_DISTANCE_SENSOR
static float s_last_range_m;
static bool  s_last_range_valid;
#endif

/* Previous flow stamp (us), used to derive the integration window because the
 * raw PixartPAA3905 proto carries no per-sample dt. */
static uint64_t s_flow_last_us;

static void map_optical_flow(const synapse_pb_PixartPAA3905 *z,
			     px4_msgs_c_SensorOpticalFlow_t *p)
{
	memset(p, 0, sizeof(*p));

	uint64_t now_us = 0;
	if (z->has_stamp) {
		now_us = stamp_to_us(&z->stamp);
		p->timestamp        = now_us;
		p->timestamp_sample = now_us;
	}
	p->device_id = 0;

	/* Integration window from the stamp delta (clamped). The very first
	 * sample yields dt=0 and is rejected by flow_sample_valid(). */
	uint32_t dt_us = 0;
	if (s_flow_last_us != 0ULL && now_us > s_flow_last_us) {
		uint64_t d = now_us - s_flow_last_us;
		if (d > FLOW_DT_MAX_US) {
			d = FLOW_DT_MAX_US;
		}
		dt_us = (uint32_t)d;
	}
	s_flow_last_us = now_us;
	p->integration_timespan_us = dt_us;

	/* Raw PAA3905 counts -> integrated angular displacement (rad).
	 * Sign on Y matches the previous FLU/sensor axis convention. */
	p->pixel_flow[0] =  (float)z->delta_x * FLOW_RAD_PER_COUNT;
	p->pixel_flow[1] = -(float)z->delta_y * FLOW_RAD_PER_COUNT;

	/* Gyro de-rotation: not supplied by the flow sensor itself. Left
	 * unavailable so EKF2 de-rotates with the FMU/vehicle gyro. To supply
	 * it here, integrate topic_imu0 over [s_flow_last_us, now_us] and set
	 * delta_angle[]/delta_angle_available = true. */
	p->delta_angle[0]        = 0.0f;
	p->delta_angle[1]        = 0.0f;
	p->delta_angle[2]        = 0.0f;
	p->delta_angle_available = false;

	/* Height above ground: reuse the latest AFBR-S50 range so EKF2 gets a
	 * time-aligned distance embedded in the flow sample. */
#if BRIDGE_TX_DISTANCE_SENSOR
	p->distance_m         = s_last_range_valid ? s_last_range_m : 0.0f;
	p->distance_available = s_last_range_valid;
#else
	p->distance_m         = 0.0f;
	p->distance_available = false;
#endif

	p->quality             = (uint8_t)MIN(z->squal, 255U);
	p->error_count         = 0;
	p->max_flow_rate       = FLOW_MAX_RATE_RAD_S;
	p->min_ground_distance = FLOW_MIN_GND_DIST_M;
	p->max_ground_distance = FLOW_MAX_GND_DIST_M;
	p->mode                = px4_flow_mode_from_pb(z->mode);
}

/* Drop samples EKF2 cannot use: a zero integration window (first sample or a
 * stalled producer) makes flow_rate = flow/dt undefined, and zero quality is
 * unusable. Mirrors the gps_sample_valid() gating pattern. */
static bool flow_sample_valid(const px4_msgs_c_SensorOpticalFlow_t *p)
{
	return p->integration_timespan_us > 0U && p->quality > 0U;
}
#endif /* BRIDGE_TX_OPTICAL_FLOW */

#if BRIDGE_TX_DISTANCE_SENSOR
static void map_distance_sensor(const synapse_pb_ArgusResults *z,
				px4_msgs_c_DistanceSensor_t *p)
{
	memset(p, 0, sizeof(*p));

	if (z->has_stamp) {
		p->timestamp = stamp_to_us(&z->stamp);
	}
	p->device_id        = 0;
	p->min_distance     = DIST_MIN_M;
	p->max_distance     = DIST_MAX_M;
	p->current_distance = z->has_bin ? z->bin.range : 0.0f;
	p->variance         = 0.0f;
	p->signal_quality   = z->has_bin ? (int8_t)MIN(z->bin.signal_quality, 100U) : (int8_t)-1;

	/* Cache range so map_optical_flow() can embed a fresh height in each
	 * SensorOpticalFlow sample. Valid only for an in-range reading. */
#if BRIDGE_TX_OPTICAL_FLOW
	s_last_range_valid = z->has_bin && p->current_distance >= DIST_MIN_M &&
			     p->current_distance <= DIST_MAX_M;
	s_last_range_m = s_last_range_valid ? p->current_distance : 0.0f;
#endif
	p->type             = PX4_MSGS_C_DISTANCE_SENSOR_TYPE_LASER;
	p->h_fov            = 0.0f;
	p->v_fov            = 0.0f;
	p->q[0]             = 1.0f;
	p->q[1]             = 0.0f;
	p->q[2]             = 0.0f;
	p->q[3]             = 0.0f;
	p->orientation      = PX4_MSGS_C_ROTATION_DOWNWARD_FACING;
	p->mode             = PX4_MSGS_C_DISTANCE_SENSOR_MODE_ENABLED;
}
#endif

#if BRIDGE_TX_SENSOR_GPS
static bool gps_sample_valid(const px4_msgs_c_SensorGps_t *p)
{
	if (p->fix_type < PX4_MSGS_C_GPS_FIX_TYPE_2D) {
		return false;
	}

	if (!isfinite(p->latitude_deg) || !isfinite(p->longitude_deg) ||
	    !isfinite(p->altitude_msl_m)) {
		return false;
	}
	if (p->latitude_deg < -90.0 || p->latitude_deg > 90.0 ||
	    p->longitude_deg < -180.0 || p->longitude_deg > 180.0) {
		return false;
	}

	if (p->latitude_deg == 0.0 && p->longitude_deg == 0.0) {
		return false;
	}
	return true;
}


static void map_sensor_gps(const synapse_pb_NavSatFix *z, px4_msgs_c_SensorGps_t *p)
{
	memset(p, 0, sizeof(*p));

	if (z->has_stamp) {
		p->timestamp        = stamp_to_us(&z->stamp);
		p->timestamp_sample = p->timestamp;
	}
	p->device_id = 0;

	p->latitude_deg         = z->latitude;
	p->longitude_deg        = z->longitude;
	p->altitude_msl_m       = z->altitude;
	p->altitude_ellipsoid_m = z->altitude;

	p->fix_type = PX4_MSGS_C_GPS_FIX_TYPE_3D;


	if (z->has_position_covariance) {
		const float ch = (float)z->position_covariance.x_x;
		const float cv = (float)z->position_covariance.z_z;
		p->eph = ch > 0.0f ? sqrtf(ch) : 0.0f;
		p->epv = cv > 0.0f ? sqrtf(cv) : 0.0f;
	}

	p->s_variance_m_s = 0.0f;
	p->c_variance_rad = 0.0f;
	p->hdop           = 0.0f;
	p->vdop           = 0.0f;
	p->vel_ned_valid  = false;
	p->heading        = NAN;
	p->jamming_state    = PX4_MSGS_C_JAMMING_STATE_UNKNOWN;
	p->spoofing_state   = PX4_MSGS_C_SPOOFING_STATE_UNKNOWN;
	p->authentication_state = PX4_MSGS_C_AUTHENTICATION_STATE_UNKNOWN;
}
#endif

#if BRIDGE_TX_SENSOR_MAG
static void map_sensor_mag(const synapse_pb_MagneticField *z, px4_msgs_c_SensorMag_t *p)
{
	memset(p, 0, sizeof(*p));

	if (z->has_stamp) {
		p->timestamp        = stamp_to_us(&z->stamp);
		p->timestamp_sample = p->timestamp;
	}
	p->device_id = 0;

	/* Tesla -> Gauss: 1 T = 1e4 G. FLU (x fwd, y left, z up) -> FRD on axes. */
	if (z->has_magnetic_field) {
		p->x = (float)z->magnetic_field.x * 1.0e4f;
		p->y = -(float)z->magnetic_field.y * 1.0e4f;
		p->z = -(float)z->magnetic_field.z * 1.0e4f;
	}
	p->temperature = 0.0f;
	p->error_count = 0;
}
#endif

#if BRIDGE_TX_SENSOR_COMBINED
/* Previous IMU stamp (us) for deriving the integration period, since
 * synapse_pb_Imu carries no per-sample dt. */
static uint64_t s_imu_last_us;

/* px4 SensorCombined integration periods are uint32 us; clamp the stamp delta. */
#define IMU_DT_MAX_US 100000U /* 100 ms */

static void map_sensor_combined(const synapse_pb_Imu *z, px4_msgs_c_SensorCombined_t *p)
{
	memset(p, 0, sizeof(*p));

	uint64_t now_us = 0;
	if (z->has_stamp) {
		now_us = stamp_to_us(&z->stamp);
		p->timestamp = now_us;
	}

	/* Integration period from the stamp delta (clamped). First sample -> 0,
	 * rejected by sensor_combined_valid(). Gyro and accel are sampled together
	 * on the ICM, so they share the same window. */
	uint32_t dt_us = 0;
	if (s_imu_last_us != 0ULL && now_us > s_imu_last_us) {
		uint64_t d = now_us - s_imu_last_us;
		if (d > IMU_DT_MAX_US) {
			d = IMU_DT_MAX_US;
		}
		dt_us = (uint32_t)d;
	}
	s_imu_last_us = now_us;

	/* FLU (x fwd, y left, z up) -> FRD (x fwd, y right, z down): negate y,z.
	 * Same axis convention as map_sensor_mag(). */
	if (z->has_angular_velocity) {
		p->gyro_rad[0] =  (float)z->angular_velocity.x;
		p->gyro_rad[1] = -(float)z->angular_velocity.y;
		p->gyro_rad[2] = -(float)z->angular_velocity.z;
	}
	p->gyro_integral_dt = dt_us;

	p->accelerometer_timestamp_relative = 0; /* accel stamped same as gyro */
	if (z->has_linear_acceleration) {
		p->accelerometer_m_s2[0] =  (float)z->linear_acceleration.x;
		p->accelerometer_m_s2[1] = -(float)z->linear_acceleration.y;
		p->accelerometer_m_s2[2] = -(float)z->linear_acceleration.z;
	}
	p->accelerometer_integral_dt = dt_us;

	p->accelerometer_clipping   = 0;
	p->gyro_clipping            = 0;
	p->accel_calibration_count  = 0;
	p->gyro_calibration_count   = 0;
}

/* Reject the first sample (dt=0) and any with a zero integration window;
 * EKF2 forms rates from integral_dt and a zero window is unusable. */
static bool sensor_combined_valid(const px4_msgs_c_SensorCombined_t *p)
{
	return p->gyro_integral_dt > 0U && p->accelerometer_integral_dt > 0U;
}
#endif

/* -------- topic table row -------- */

struct tx_row {
	struct zros_topic *ztopic;
	void              *zbuf;        /* ZROS scratch */
	void              *pbuf;        /* px4 struct scratch */
	uint8_t           *cdr_buf;
	size_t             cdr_size;
	void             (*map_fn)(const void *zsrc, void *pdst);
	int              (*encode_fn)(struct ucdrBuffer *ub, const void *pdst);
	bool             (*valid_fn)(const void *pdst);
	const char        *topic_leaf; /* "sensor_optical_flow" */
	const char        *ros_type;   /* "px4_msgs::msg::dds_::SensorOpticalFlow_" */
	const char        *rihs_hash;  /* full "RIHS01_..." */
	int32_t            rate_hz;
	z_owned_publisher_t publisher;
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	int64_t                    seq;   /* rmw_zenoh per-publisher sample counter */
	uint32_t                   eid;   /* rmw_zenoh entity id (unique per row) */
	z_owned_liveliness_token_t token; /* graph-discovery liveliness token */
#endif
};

/* Thin wrappers so the void* map_fn/encode_fn table stays type-clean. */
#if BRIDGE_TX_OPTICAL_FLOW
static void map_flow_thunk(const void *z, void *p)
{
	map_optical_flow((const synapse_pb_PixartPAA3905 *)z,
			 (px4_msgs_c_SensorOpticalFlow_t *)p);
}
static int enc_flow_thunk(struct ucdrBuffer *ub, const void *p)
{
	return px4_msgs_c_encode_sensor_optical_flow(ub,
			(const px4_msgs_c_SensorOpticalFlow_t *)p);
}
static bool valid_flow_thunk(const void *p)
{
	return flow_sample_valid((const px4_msgs_c_SensorOpticalFlow_t *)p);
}
#endif
#if BRIDGE_TX_DISTANCE_SENSOR
static void map_dist_thunk(const void *z, void *p)
{
	map_distance_sensor((const synapse_pb_ArgusResults *)z,
			    (px4_msgs_c_DistanceSensor_t *)p);
}
static int enc_dist_thunk(struct ucdrBuffer *ub, const void *p)
{
	return px4_msgs_c_encode_distance_sensor(ub,
			(const px4_msgs_c_DistanceSensor_t *)p);
}
#endif
#if BRIDGE_TX_SENSOR_GPS
static void map_gps_thunk(const void *z, void *p)
{
	map_sensor_gps((const synapse_pb_NavSatFix *)z,
		       (px4_msgs_c_SensorGps_t *)p);
}
static int enc_gps_thunk(struct ucdrBuffer *ub, const void *p)
{
	return px4_msgs_c_encode_sensor_gps(ub, (const px4_msgs_c_SensorGps_t *)p);
}
static bool valid_gps_thunk(const void *p)
{
	return gps_sample_valid((const px4_msgs_c_SensorGps_t *)p);
}
#endif
#if BRIDGE_TX_SENSOR_MAG
static void map_mag_thunk(const void *z, void *p)
{
	map_sensor_mag((const synapse_pb_MagneticField *)z,
		       (px4_msgs_c_SensorMag_t *)p);
}
static int enc_mag_thunk(struct ucdrBuffer *ub, const void *p)
{
	return px4_msgs_c_encode_sensor_mag(ub, (const px4_msgs_c_SensorMag_t *)p);
}
#endif
#if BRIDGE_TX_SENSOR_COMBINED
static void map_imu_thunk(const void *z, void *p)
{
	map_sensor_combined((const synapse_pb_Imu *)z,
			    (px4_msgs_c_SensorCombined_t *)p);
}
static int enc_imu_thunk(struct ucdrBuffer *ub, const void *p)
{
	return px4_msgs_c_encode_sensor_combined(ub,
			(const px4_msgs_c_SensorCombined_t *)p);
}
static bool valid_imu_thunk(const void *p)
{
	return sensor_combined_valid((const px4_msgs_c_SensorCombined_t *)p);
}
#endif

#if BRIDGE_TX_COUNT > 0
static struct tx_row tx_table[] = {
#if BRIDGE_TX_OPTICAL_FLOW
	{
		.ztopic     = &topic_optical_flow_raw,
		.zbuf       = &g_ctx.z_flow,
		.pbuf       = &g_ctx.p_flow,
		.cdr_buf    = g_ctx.cdr_flow,
		.cdr_size   = sizeof(g_ctx.cdr_flow),
		.map_fn     = map_flow_thunk,
		.encode_fn  = enc_flow_thunk,
		.valid_fn   = valid_flow_thunk,
		.topic_leaf = PX4_MSGS_C_TOPIC_SENSOR_OPTICAL_FLOW,
		.ros_type   = PX4_MSGS_C_ROS_TYPE_SENSOR_OPTICAL_FLOW,
		.rihs_hash  = PX4_MSGS_C_RIHS_SENSOR_OPTICAL_FLOW,
		.rate_hz    = CONFIG_SYNAPSE_ZENOH_PX4_OPTICAL_FLOW_RATE_HZ,
	},
#endif
#if BRIDGE_TX_DISTANCE_SENSOR
	{
		.ztopic     = &topic_argus,
		.zbuf       = &g_ctx.z_dist_src,
		.pbuf       = &g_ctx.p_dist,
		.cdr_buf    = g_ctx.cdr_dist,
		.cdr_size   = sizeof(g_ctx.cdr_dist),
		.map_fn     = map_dist_thunk,
		.encode_fn  = enc_dist_thunk,
		.topic_leaf = PX4_MSGS_C_TOPIC_DISTANCE_SENSOR,
		.ros_type   = PX4_MSGS_C_ROS_TYPE_DISTANCE_SENSOR,
		.rihs_hash  = PX4_MSGS_C_RIHS_DISTANCE_SENSOR,
		.rate_hz    = CONFIG_SYNAPSE_ZENOH_PX4_DISTANCE_SENSOR_RATE_HZ,
	},
#endif
#if BRIDGE_TX_SENSOR_GPS
	{
		.ztopic     = &topic_nav_sat_fix,
		.zbuf       = &g_ctx.z_gnss,
		.pbuf       = &g_ctx.p_gps,
		.cdr_buf    = g_ctx.cdr_gps,
		.cdr_size   = sizeof(g_ctx.cdr_gps),
		.map_fn     = map_gps_thunk,
		.encode_fn  = enc_gps_thunk,
		.valid_fn   = valid_gps_thunk,
		.topic_leaf = PX4_MSGS_C_TOPIC_SENSOR_GPS,
		.ros_type   = PX4_MSGS_C_ROS_TYPE_SENSOR_GPS,
		.rihs_hash  = PX4_MSGS_C_RIHS_SENSOR_GPS,
		.rate_hz    = CONFIG_SYNAPSE_ZENOH_PX4_GNSS_RATE_HZ,
	},
#endif
#if BRIDGE_TX_SENSOR_MAG
	{
		.ztopic     = &topic_magnetic_field,
		.zbuf       = &g_ctx.z_mag,
		.pbuf       = &g_ctx.p_mag,
		.cdr_buf    = g_ctx.cdr_mag,
		.cdr_size   = sizeof(g_ctx.cdr_mag),
		.map_fn     = map_mag_thunk,
		.encode_fn  = enc_mag_thunk,
		.topic_leaf = PX4_MSGS_C_TOPIC_SENSOR_MAG,
		.ros_type   = PX4_MSGS_C_ROS_TYPE_SENSOR_MAG,
		.rihs_hash  = PX4_MSGS_C_RIHS_SENSOR_MAG,
		.rate_hz    = CONFIG_SYNAPSE_ZENOH_PX4_MAG_RATE_HZ,
	},
#endif
#if BRIDGE_TX_SENSOR_COMBINED
	{
		.ztopic     = &topic_imu0,
		.zbuf       = &g_ctx.z_imu,
		.pbuf       = &g_ctx.p_imu,
		.cdr_buf    = g_ctx.cdr_imu,
		.cdr_size   = sizeof(g_ctx.cdr_imu),
		.map_fn     = map_imu_thunk,
		.encode_fn  = enc_imu_thunk,
		.valid_fn   = valid_imu_thunk,
		.topic_leaf = PX4_MSGS_C_TOPIC_SENSOR_COMBINED,
		.ros_type   = PX4_MSGS_C_ROS_TYPE_SENSOR_COMBINED,
		.rihs_hash  = PX4_MSGS_C_RIHS_SENSOR_COMBINED,
		.rate_hz    = CONFIG_SYNAPSE_ZENOH_PX4_SENSOR_COMBINED_RATE_HZ,
	},
#endif
};
BUILD_ASSERT(ARRAY_SIZE(tx_table) == BRIDGE_TX_COUNT);
static struct zros_sub tx_subs[BRIDGE_TX_COUNT];

#if BRIDGE_TX_OPTICAL_FLOW
BUILD_ASSERT(CDR_BUF_MAX >= PX4_MSGS_C_SENSOR_OPTICAL_FLOW_CDR_MAX,
	     "CDR_BUF_MAX too small for SensorOpticalFlow");
#endif
#if BRIDGE_TX_DISTANCE_SENSOR
BUILD_ASSERT(CDR_BUF_MAX >= PX4_MSGS_C_DISTANCE_SENSOR_CDR_MAX,
	     "CDR_BUF_MAX too small for DistanceSensor");
#endif
#if BRIDGE_TX_SENSOR_GPS
BUILD_ASSERT(CDR_BUF_MAX >= PX4_MSGS_C_SENSOR_GPS_CDR_MAX,
	     "CDR_BUF_MAX too small for SensorGps");
#endif
#if BRIDGE_TX_SENSOR_MAG
BUILD_ASSERT(CDR_BUF_MAX >= PX4_MSGS_C_SENSOR_MAG_CDR_MAX,
	     "CDR_BUF_MAX too small for SensorMag");
#endif
#if BRIDGE_TX_SENSOR_COMBINED
BUILD_ASSERT(CDR_BUF_MAX >= PX4_MSGS_C_SENSOR_COMBINED_CDR_MAX,
	     "CDR_BUF_MAX too small for SensorCombined");
#endif
#endif /* BRIDGE_TX_COUNT > 0 */

#define PX4_MSGS_C_TYPE_HASH_NOT_SUPPORTED "TypeHashNotSupported"

static int build_keyexpr(char *out, size_t out_size, const char *topic_leaf,
			 const char *ros_type, const char *rihs_hash)
{
	const char *ns = CONFIG_SYNAPSE_ZENOH_PX4_NAMESPACE;
	const int domain_id = CONFIG_SYNAPSE_ZENOH_PX4_DOMAIN_ID;
	int len;

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RIHS_HASH)
	const char *hash_seg = rihs_hash;
#else
	const char *hash_seg = PX4_MSGS_C_TYPE_HASH_NOT_SUPPORTED;
	ARG_UNUSED(rihs_hash);
#endif

	if (ns[0] != '\0') {
		len = snprintf(out, out_size, "%d/%s/fmu/in/%s/%s/%s", domain_id, ns,
			       topic_leaf, ros_type, hash_seg);
	} else {
		len = snprintf(out, out_size, "%d/fmu/in/%s/%s/%s", domain_id, topic_leaf,
			       ros_type, hash_seg);
	}

	return (len > 0 && (size_t)len < out_size) ? 0 : -EINVAL;
}

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)

#define RMW_LIVELINESS_KEYEXPR_MAX 512

static inline bool rmw_compat_active(void)
{
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG)
	return g_run_cfg.mode == ZPX4_MODE_CLIENT;
#elif defined(CONFIG_SYNAPSE_ZENOH_PX4_MODE_CLIENT)
	return true;
#else
	return false;
#endif
}

/* rmw_zenoh keyexpr-mangles '/' in names to '%'. Copy src into dst applying
 * that substitution; dst must hold strlen(src)+1 bytes. */
static void rmw_mangle_name(char *dst, size_t dst_size, const char *src)
{
	size_t j = 0;

	for (size_t i = 0; src[i] != '\0' && j + 1 < dst_size; i++) {
		dst[j++] = (src[i] == '/') ? '%' : src[i];
	}
	dst[j] = '\0';
}

static void rmw_capture_source_gid(struct context *ctx)
{
	z_id_t zid = z_info_zid(z_loan(ctx->sessions[0]));

	BUILD_ASSERT(sizeof(zid.id) == sizeof(ctx->source_gid));
	memcpy(ctx->source_gid, zid.id, sizeof(ctx->source_gid));

	static const char hexd[] = "0123456789abcdef";
	for (size_t i = 0; i < sizeof(zid.id); i++) {
		ctx->zid_hex[2 * i]     = hexd[(zid.id[i] >> 4) & 0xF];
		ctx->zid_hex[2 * i + 1] = hexd[zid.id[i] & 0xF];
	}
	ctx->zid_hex[2 * sizeof(zid.id)] = '\0';
}

static int rmw_build_liveliness_keyexpr(char *out, size_t out_size, struct context *ctx,
					const struct tx_row *row)
{
	char topic_full[96];
	char topic_mangled[96];
	char type_mangled[128];
	const char *node_ns = CONFIG_SYNAPSE_ZENOH_PX4_RMW_NODE_NS;
	const char *node_name = CONFIG_SYNAPSE_ZENOH_PX4_RMW_NODE_NAME;
	const char *enclave = CONFIG_SYNAPSE_ZENOH_PX4_RMW_ENCLAVE;
	const char *qos = CONFIG_SYNAPSE_ZENOH_PX4_RMW_QOS;
	char node_ns_mangled[64];
	int len;

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RIHS_HASH)
	const char *type_hash = row->rihs_hash;
#else
	const char *type_hash = PX4_MSGS_C_TYPE_HASH_NOT_SUPPORTED;
#endif

	(void)snprintf(topic_full, sizeof(topic_full), "/fmu/in/%s", row->topic_leaf);
	rmw_mangle_name(topic_mangled, sizeof(topic_mangled), topic_full);
	rmw_mangle_name(type_mangled, sizeof(type_mangled), row->ros_type);
	rmw_mangle_name(node_ns_mangled, sizeof(node_ns_mangled), node_ns);

	len = snprintf(out, out_size,
		       "@ros2_lv/%d/%s/0/%u/MP/%s/%s/%s/%s/%s/%s/%s",
		       CONFIG_SYNAPSE_ZENOH_PX4_DOMAIN_ID, ctx->zid_hex, row->eid,
		       enclave, node_ns_mangled, node_name, topic_mangled, type_mangled,
		       type_hash, qos);

	return (len > 0 && (size_t)len < out_size) ? 0 : -EINVAL;
}

static int rmw_build_node_keyexpr(char *out, size_t out_size, struct context *ctx)
{
	const char *node_ns = CONFIG_SYNAPSE_ZENOH_PX4_RMW_NODE_NS;
	const char *node_name = CONFIG_SYNAPSE_ZENOH_PX4_RMW_NODE_NAME;
	const char *enclave = CONFIG_SYNAPSE_ZENOH_PX4_RMW_ENCLAVE;
	char node_ns_mangled[64];
	int len;

	rmw_mangle_name(node_ns_mangled, sizeof(node_ns_mangled), node_ns);

	len = snprintf(out, out_size, "@ros2_lv/%d/%s/0/0/NN/%s/%s/%s",
		       CONFIG_SYNAPSE_ZENOH_PX4_DOMAIN_ID, ctx->zid_hex, enclave,
		       node_ns_mangled, node_name);

	return (len > 0 && (size_t)len < out_size) ? 0 : -EINVAL;
}

static int rmw_build_attachment(struct context *ctx, struct tx_row *row, z_owned_bytes_t *out)
{
	ze_owned_serializer_t ser;
	uint64_t sample_us;

	memcpy(&sample_us, row->pbuf, sizeof(sample_us));
	int64_t ts_ns = (sample_us != 0)
				? (int64_t)(sample_us * 1000ULL)
				: (int64_t)k_ticks_to_ns_floor64(k_uptime_ticks());

	if (ze_serializer_empty(&ser) < 0) {
		return -EINVAL;
	}

	if (ze_serializer_serialize_int64(z_loan_mut(ser), row->seq++) < 0 ||
	    ze_serializer_serialize_int64(z_loan_mut(ser), ts_ns) < 0 ||
	    ze_serializer_serialize_buf(z_loan_mut(ser), ctx->source_gid,
					sizeof(ctx->source_gid)) < 0) {
		z_drop(z_move(ser));
		return -EINVAL;
	}
	ze_serializer_finish(z_move(ser), out);
	return 0;
}
#endif /* CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT */

/* -------- session -------- */

static int session_open(struct context *ctx)
{
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG)

	const bool is_peer = (g_run_cfg.mode == ZPX4_MODE_PEER);
	const char *mode = is_peer ? "peer" : "client";
	const uint8_t endpoint_key = is_peer ? Z_CONFIG_LISTEN_KEY : Z_CONFIG_CONNECT_KEY;
	const char *endpoint_desc = is_peer ? "listen" : "connect";
	const char *locator = g_run_cfg.locator;
#elif defined(CONFIG_SYNAPSE_ZENOH_PX4_MODE_PEER)
	const char *mode = "peer";
	const uint8_t endpoint_key = Z_CONFIG_LISTEN_KEY;
	const char *endpoint_desc = "listen";
	const char *locator = CONFIG_SYNAPSE_ZENOH_PX4_LOCATOR;
#else
	const char *mode = "client";
	const uint8_t endpoint_key = Z_CONFIG_CONNECT_KEY;
	const char *endpoint_desc = "connect";
	const char *locator = CONFIG_SYNAPSE_ZENOH_PX4_LOCATOR;
#endif
	z_owned_config_t config;
	int ret;

	LOG_INF("Opening PX4 zenoh session (%s, %s %s) ...", mode, endpoint_desc, locator);

	while (true) {
		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);
		zp_config_insert(z_loan_mut(config), endpoint_key, locator);

		ret = z_open(&ctx->sessions[0], z_move(config), NULL);
		if (ret == 0) {
			break;
		}
		LOG_WRN("z_open failed (%d), retrying...", ret);
		if (k_sem_count_get(&ctx->running) != 0) {
			(void)k_sem_take(&ctx->running, K_NO_WAIT);
			LOG_INF("session open aborted by stop request");
			return -ECANCELED;
		}
		k_sleep(K_SECONDS(5));

	}


#if Z_FEATURE_MULTI_THREAD == 1
	if (zp_start_read_task(z_loan_mut(ctx->sessions[0]), NULL) < 0 ||
	    zp_start_lease_task(z_loan_mut(ctx->sessions[0]), NULL) < 0) {
		LOG_ERR("start read/lease tasks failed");
		z_drop(z_move(ctx->sessions[0]));
		return -EINVAL;
	}
#endif

	LOG_INF("PX4 zenoh session open");
	return 0;
}

#if BRIDGE_TX_COUNT > 0
static int declare_publishers(struct context *ctx)
{
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	ctx->rmw_declared = rmw_compat_active();
	if (ctx->rmw_declared) {
		rmw_capture_source_gid(ctx);

		char nke_str[RMW_LIVELINESS_KEYEXPR_MAX];
		z_view_keyexpr_t nke;

		if (rmw_build_node_keyexpr(nke_str, sizeof(nke_str), ctx) < 0) {
			LOG_ERR("node keyexpr too long");
			return -EINVAL;
		}
		if (z_view_keyexpr_from_str(&nke, nke_str) < 0) {
			LOG_ERR("bad node keyexpr %s", nke_str);
			return -EINVAL;
		}
		if (z_liveliness_declare_token(z_loan(ctx->sessions[0]), &ctx->node_token,
					       z_loan(nke), NULL) < 0) {
			LOG_ERR("node liveliness token declare failed");
			return -EINVAL;
		}
		LOG_INF("PX4 zenoh node liveliness %s", nke_str);
	}
#endif

	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		char keyexpr[KEYEXPR_MAX];
		z_view_keyexpr_t ke;
		z_owned_encoding_t enc;
		z_publisher_options_t opts;
		int ret;

		ret = build_keyexpr(keyexpr, sizeof(keyexpr), tx_table[i].topic_leaf,
				    tx_table[i].ros_type, tx_table[i].rihs_hash);
		if (ret < 0) {
			LOG_ERR("keyexpr too long: %s", tx_table[i].topic_leaf);
			return ret;
		}

		if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
			LOG_ERR("bad keyexpr %s", keyexpr);
			return -EINVAL;
		}
		z_publisher_options_default(&opts);

		opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
#ifdef Z_FEATURE_UNSTABLE_API
		opts.reliability = Z_RELIABILITY_BEST_EFFORT;
#endif

		if (z_encoding_from_str(&enc, CDR_ENCODING_STR) == 0) {
			opts.encoding = z_move(enc);
		}
		if (z_declare_publisher(z_loan(ctx->sessions[0]), &tx_table[i].publisher,
					z_loan(ke), &opts) < 0) {
			LOG_ERR("declare_publisher failed for %s", keyexpr);
			return -EINVAL;
		}
		LOG_INF("PX4 zenoh publisher %s", keyexpr);

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
		if (ctx->rmw_declared) {
			char lke_str[RMW_LIVELINESS_KEYEXPR_MAX];
			z_view_keyexpr_t lke;

			tx_table[i].eid = (uint32_t)(i + 1);
			tx_table[i].seq = 0;

			if (rmw_build_liveliness_keyexpr(lke_str, sizeof(lke_str), ctx,
							 &tx_table[i]) < 0) {
				LOG_ERR("liveliness keyexpr too long: %s",
					tx_table[i].topic_leaf);
				return -EINVAL;
			}
			if (z_view_keyexpr_from_str(&lke, lke_str) < 0) {
				LOG_ERR("bad liveliness keyexpr %s", lke_str);
				return -EINVAL;
			}
			if (z_liveliness_declare_token(z_loan(ctx->sessions[0]),
						       &tx_table[i].token, z_loan(lke),
						       NULL) < 0) {
				LOG_ERR("liveliness token declare failed for %s",
					tx_table[i].topic_leaf);
				return -EINVAL;
			}
			LOG_INF("PX4 zenoh liveliness %s", lke_str);
		}
#endif
	}
	return 0;
}

static int publish_cdr(struct tx_row *row)
{
	struct ucdrBuffer ub;
	z_owned_bytes_t payload;
	z_publisher_put_options_t opts;

	ucdr_init_buffer(&ub, row->cdr_buf, row->cdr_size);
	if (row->encode_fn(&ub, row->pbuf) != 0) {
		LOG_WRN("CDR encode failed for %s", row->topic_leaf);
		return -EMSGSIZE;
	}

	const size_t len = (size_t)(ub.iterator - ub.init);

	if (z_bytes_from_static_buf(&payload, row->cdr_buf, len) < 0) {
		return -ENOMEM;
	}
	z_publisher_put_options_default(&opts);
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	if (g_ctx.rmw_declared) {
		z_owned_bytes_t attachment;
		if (rmw_build_attachment(&g_ctx, row, &attachment) == 0) {
			opts.attachment = z_move(attachment);
		}
	}
#endif
	if (z_publisher_put(z_loan(row->publisher), z_move(payload), &opts) < 0) {
		return -EIO;
	}
	return 0;
}

#endif /* BRIDGE_TX_COUNT > 0 */

/* -------- bridge lifecycle -------- */

static int bridge_init(struct context *ctx)
{
	k_sem_take(&ctx->running, K_FOREVER);
	zros_node_init(&ctx->node, "zenoh_px4");

#if BRIDGE_TX_COUNT > 0
	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		int ret = zros_sub_init(&tx_subs[i], &ctx->node, tx_table[i].ztopic,
					tx_table[i].zbuf, tx_table[i].rate_hz);
		if (ret < 0) {
			LOG_ERR("zros_sub_init %zu failed: %d", i, ret);
			return ret;
		}
	}
#endif

	int ret = session_open(ctx);
	if (ret < 0) {
		return ret;
	}

#if BRIDGE_TX_COUNT > 0
	ret = declare_publishers(ctx);
	if (ret < 0) {
		return ret;
	}
#endif

	LOG_INF("init");
	return 0;
}


static void bridge_fini(struct context *ctx)
{
#if BRIDGE_TX_COUNT > 0
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	if (ctx->rmw_declared) {
		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			z_liveliness_undeclare_token(z_move(tx_table[i].token));
		}
		z_liveliness_undeclare_token(z_move(ctx->node_token));
	}
#endif

	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		z_undeclare_publisher(z_move(tx_table[i].publisher));
	}
	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		zros_sub_fini(&tx_subs[i]);
	}
#endif
	z_drop(z_move(ctx->sessions[0]));
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

#if BRIDGE_TX_COUNT > 0

static void session_teardown(struct context *ctx)
{
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RMW_COMPAT)
	if (ctx->rmw_declared) {
		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			z_liveliness_undeclare_token(z_move(tx_table[i].token));
		}
		z_liveliness_undeclare_token(z_move(ctx->node_token));
		ctx->rmw_declared = false;
	}
#endif
	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		z_undeclare_publisher(z_move(tx_table[i].publisher));
	}
	z_drop(z_move(ctx->sessions[0]));
}

static int bridge_reconnect(struct context *ctx)
{
	int ret;

	LOG_WRN("session lost, reconnecting...");
	session_teardown(ctx);

	ret = session_open(ctx);
	if (ret < 0) {
		return ret;
	}
	ret = declare_publishers(ctx);
	if (ret < 0) {
		return ret;
	}
	LOG_INF("reconnected");
	return 0;
}
#endif /* BRIDGE_TX_COUNT > 0 */

static void bridge_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	if (bridge_init(ctx) < 0) {
		bridge_fini(ctx);
		return;
	}

#if BRIDGE_TX_COUNT > 0
	struct k_poll_event events[ARRAY_SIZE(tx_table)];
	unsigned int put_fails = 0;
	int64_t last_reconnect_ms = 0; /* uptime of last reconnect attempt */

	for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
		events[i] = *zros_sub_get_event(&tx_subs[i]);
	}
#endif

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

#if BRIDGE_TX_COUNT > 0
		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
		}
		(void)k_poll(events, ARRAY_SIZE(events), K_MSEC(POLL_MS));

		for (size_t i = 0; i < ARRAY_SIZE(tx_table); i++) {
			if (zros_sub_update(&tx_subs[i]) == 0) {
				tx_table[i].map_fn(tx_table[i].zbuf, tx_table[i].pbuf);
				if (tx_table[i].valid_fn != NULL &&
				    !tx_table[i].valid_fn(tx_table[i].pbuf)) {
					continue;
				}
				if (publish_cdr(&tx_table[i]) == -EIO) {
					if (put_fails == 0) {
						LOG_WRN("publisher_put failed for %s",
							tx_table[i].topic_leaf);
					}
					put_fails++;
				} else {
					put_fails = 0;
				}
			}

		}

		if (put_fails >= PUT_FAIL_LIMIT) {
			/* Rate-limit reconnects: a storm of teardown/reopen never lets
			 * the far side re-discover the graph. Only actually reconnect
			 * once per cooldown; otherwise back off and keep trying puts so
			 * a transient recovers on its own without a session churn. */
			int64_t now_ms = k_uptime_get();

			if ((now_ms - last_reconnect_ms) >= RECONNECT_COOLDOWN_MS) {
				last_reconnect_ms = now_ms;
				if (bridge_reconnect(ctx) < 0) {
					break;
				}
				put_fails = 0;
			} else {
				/* within cooldown: don't churn the session. Clamp the
				 * counter just below the limit so we re-evaluate next
				 * iteration instead of reconnecting every loop. */
				put_fails = PUT_FAIL_LIMIT - 1;
				k_sleep(K_MSEC(POLL_MS));
			}
		}
#else
		k_sleep(K_MSEC(POLL_MS));
#endif
	}


	bridge_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, g_stack, K_THREAD_STACK_SIZEOF(g_stack),
				      bridge_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "zenoh_px4");
	k_thread_start(tid);
	return 0;
}

static int bridge_stop_sync(struct context *ctx)
{
	if (k_sem_count_get(&ctx->running) != 0) {
		return -EALREADY;
	}

	k_sem_give(&ctx->running);
	(void)k_thread_join(&ctx->thread_data, K_SECONDS(10));
	return 0;
}

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG)

static int zpx4_config_net(const struct shell *sh, uint8_t mode, const char *arg)
{
	int n;

	if (strncmp(arg, "tcp/", 4) != 0 && strncmp(arg, "udp/", 4) != 0) {
		shell_error(sh,
			    "locator must start with 'tcp/' or 'udp/' (e.g. tcp/10.58.0.1:7447)");
		return -EINVAL;
	}

	n = snprintf(g_run_cfg.locator, sizeof(g_run_cfg.locator), "%s", arg);
	if (n < 0 || (size_t)n >= sizeof(g_run_cfg.locator)) {
		shell_error(sh, "locator too long");
		return -EINVAL;
	}
	g_run_cfg.mode = mode;

	shell_print(sh, "staged %s %s (run `zenoh_px4 restart` to apply)",
		    mode == ZPX4_MODE_PEER ? "peer" : "client", g_run_cfg.locator);
	return 0;
}

static int cmd_config_net_peer(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return zpx4_config_net(sh, ZPX4_MODE_PEER, argv[1]);
}

static int cmd_config_net_client(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	return zpx4_config_net(sh, ZPX4_MODE_CLIENT, argv[1]);
}

static int cmd_config_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "mode:    %s", g_run_cfg.mode == ZPX4_MODE_PEER ? "peer" : "client");
	shell_print(sh, "locator: %s", g_run_cfg.locator);
	shell_print(sh, "running: %d", (int)(k_sem_count_get(&g_ctx.running) == 0));
	return 0;
}

#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_PERSIST)

static int cmd_config_save(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Persisting erases/programs the internal flash the CPU runs from (XIP),
	 * which briefly stalls the core. Require the bridge stopped so the save
	 * cannot disturb a live zenoh session.
	 */
	if (k_sem_count_get(&g_ctx.running) == 0) {
		shell_error(sh,
			    "bridge is running; run `zenoh_px4 stop` before `config save`");
		return -EBUSY;
	}
	if (!zpx4_locator_ok(g_run_cfg.locator)) {
		shell_error(sh, "refusing to save invalid locator '%s'", g_run_cfg.locator);
		return -EINVAL;
	}
	if (zpx4_nvs_mount() != 0) {
		shell_error(sh, "storage not available");
		return -EIO;
	}

	struct zpx4_persist rec = {
		.mode = g_run_cfg.mode,
	};
	(void)snprintf(rec.locator, sizeof(rec.locator), "%s", g_run_cfg.locator);

	ssize_t rc = nvs_write(&g_nvs, ZPX4_CFG_ID, &rec, sizeof(rec));

	if (rc < 0) {
		shell_error(sh, "save failed (%d)", (int)rc);
		return (int)rc;
	}
	shell_print(sh, "saved (%s %s); reboot or `restart` to apply",
		    rec.mode == ZPX4_MODE_PEER ? "peer" : "client", rec.locator);
	return 0;
}

static int cmd_config_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (zpx4_nvs_mount() != 0) {
		shell_error(sh, "storage not available");
		return -EIO;
	}

	int rc = nvs_delete(&g_nvs, ZPX4_CFG_ID);

	if (rc) {
		shell_error(sh, "clear failed (%d)", rc);
		return rc;
	}
	shell_print(sh, "cleared; build defaults restored on next boot");
	return 0;
}

#endif /* CONFIG_SYNAPSE_ZENOH_PX4_PERSIST */

static int cmd_restart(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	(void)bridge_stop_sync(&g_ctx);
	start(&g_ctx);
	shell_print(sh, "restarted (%s %s)",
		    g_run_cfg.mode == ZPX4_MODE_PEER ? "peer" : "client", g_run_cfg.locator);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config_net,
	SHELL_CMD_ARG(peer, NULL, "peer {tcp|udp}/<ip>:<port> (node LISTENs on this local addr)",
            cmd_config_net_peer, 2, 0),
	SHELL_CMD_ARG(client, NULL, "client {tcp|udp}/<ip>:<port> (connect to zenoh router)",
            cmd_config_net_client, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
	SHELL_CMD(net, &sub_config_net, "set net mode + locator", NULL),
	SHELL_CMD(show, NULL, "show staged/active config", cmd_config_show),
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_PERSIST)
	SHELL_CMD(save, NULL, "persist config to flash (bridge must be stopped)",
		  cmd_config_save),
	SHELL_CMD(clear, NULL, "erase persisted config (revert to defaults)",
		  cmd_config_clear),
#endif
	SHELL_SUBCMD_SET_END);

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (k_sem_count_get(&g_ctx.running) == 0) {
		shell_print(sh, "already running");
	} else {
		start(&g_ctx);
	}
	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (bridge_stop_sync(&g_ctx) < 0) {
		shell_print(sh, "not running");
	}
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "running: %d", (int)(k_sem_count_get(&g_ctx.running) == 0));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_zenoh_px4,
	SHELL_CMD(start, NULL, "start bridge", cmd_start),
	SHELL_CMD(stop, NULL, "stop bridge", cmd_stop),
	SHELL_CMD(status, NULL, "bridge status", cmd_status),
	SHELL_CMD(config, &sub_config, "runtime net config", NULL),
	SHELL_CMD(restart, NULL, "reopen session with current config", cmd_restart),
	SHELL_SUBCMD_SET_END);

#else /* !CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG */

static int zenoh_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&ctx->running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (bridge_stop_sync(ctx) < 0) {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)(k_sem_count_get(&ctx->running) == 0));
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_zenoh_px4, zenoh_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));


#endif /* CONFIG_SYNAPSE_ZENOH_PX4_RUNTIME_CONFIG */

SHELL_CMD_REGISTER(zenoh_px4, &sub_zenoh_px4, "PX4 zenoh CDR egress", NULL);

static int zenoh_px4_sys_init(void)
{
#if IS_ENABLED(CONFIG_SYNAPSE_ZENOH_PX4_PERSIST)
	/* Load saved config (or keep build defaults) before the bridge starts. */
	zpx4_persist_load();
#endif
	return start(&g_ctx);
}

SYS_INIT(zenoh_px4_sys_init, APPLICATION, 92);

/* vi: ts=4 sw=4 et */