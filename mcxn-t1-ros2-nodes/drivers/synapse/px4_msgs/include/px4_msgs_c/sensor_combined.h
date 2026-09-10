/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PX4_MSGS_C_SENSOR_COMBINED_H
#define PX4_MSGS_C_SENSOR_COMBINED_H

#include <stddef.h>
#include <stdint.h>

/*
 * Mirror of px4_msgs/msg/SensorCombined (PX4 SensorCombined.msg).
 *
 * All vectors are in the PX4 FRD body frame (x-forward, y-right, z-down),
 * SI units. Message CONSTANTS (RELATIVE_TIMESTAMP_INVALID, CLIPPING_*) are
 * NOT part of the CDR wire payload and are therefore not stored here.
 *
 * NOTE: this is the IMU topic EKF2 consumes in SINGLE-instance mode
 * (SENS_IMU_MODE=1); in multi-EKF mode (SENS_IMU_MODE=0) EKF2 uses
 * vehicle_imu instead. On a flight controller that already runs its own
 * on-board IMU driver, an externally injected sensor_combined competes with
 * the local publisher; feeding EKF2 this way is only clean on a PX4 with no
 * local IMU (replay/HIL-style). It is always usable for visualization.
 */
struct px4_msgs_c_SensorCombined {
	uint64_t timestamp;                         /* uint64 timestamp (us) */
	float    gyro_rad[3];                       /* float32[3] gyro_rad (rad/s, FRD) */
	uint32_t gyro_integral_dt;                  /* uint32 gyro_integral_dt (us) */
	int32_t  accelerometer_timestamp_relative;  /* int32 */
	float    accelerometer_m_s2[3];             /* float32[3] accel (m/s^2, FRD) */
	uint32_t accelerometer_integral_dt;         /* uint32 (us) */
	uint8_t  accelerometer_clipping;            /* uint8 bitfield */
	uint8_t  gyro_clipping;                     /* uint8 bitfield */
	uint8_t  accel_calibration_count;           /* uint8 */
	uint8_t  gyro_calibration_count;            /* uint8 */
};
typedef struct px4_msgs_c_SensorCombined px4_msgs_c_SensorCombined_t;

/* Header(4) + u64(8) + pad + 3f32(12) + u32(4) + i32(4) + 3f32(12) + u32(4) +
 * 4xu8(4) plus CDR alignment slack. 96 is comfortably above the worst case. */
#define PX4_MSGS_C_SENSOR_COMBINED_CDR_MAX 96U

struct ucdrBuffer;
int px4_msgs_c_encode_sensor_combined(struct ucdrBuffer *ub,
				      const px4_msgs_c_SensorCombined_t *m);

#endif /* PX4_MSGS_C_SENSOR_COMBINED_H */
