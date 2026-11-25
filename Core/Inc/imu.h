/**
 * @file    imu.h
 * @brief   Yaw gyro: HiLetgo HW-123 (GY-521, MPU-6050) on I2C1.
 *
 * Wiring (board VCC from 3V3 or 5V, it has its own regulator):
 *   HW-123 SCL -> D15 (PB8, I2C1_SCL)     HW-123 SDA -> D14 (PB9, I2C1_SDA)
 *   HW-123 GND -> GND                     AD0 open (address 0x68), INT unused
 *
 * Only the Z gyro is read (every IMU_SAMPLE_US, at 1 kHz internal rate with
 * the chip's 42 Hz low-pass). Its rate is integrated into a yaw angle
 * (degrees, + = counter-clockwise seen from above, like turn angles). The
 * gyro's zero-rate offset (bias) is learnt whenever the caller says the
 * robot is standing still, so there is no blocking calibration.
 *
 * Failures never block for long: a read that fails marks the IMU down, and
 * it is re-probed every IMU_RETRY_MS. imu_ok() says whether the yaw can be
 * trusted right now; drive.c falls back to the encoders when it can't.
 * All tunables live in robot_config.h.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool     present;        /* answered and configured, no read failures since */
  bool     calibrated;     /* a bias has been learnt */
  uint8_t  who_am_i;       /* 0x68 = MPU-6050; clones may say 0x70/0x72/0x98 */
  float    rate_dps;       /* last yaw rate, bias removed, scaled, signed */
  float    raw_dps;        /* last raw Z rate (no bias, no scale, no sign) */
  float    bias_dps;       /* learnt zero-rate offset, raw units */
  double   yaw_deg;        /* integrated yaw since boot / imu_zero() */
  uint32_t samples;        /* successful reads */
  uint32_t errors;         /* failed I2C transactions */
  uint32_t bias_updates;   /* still windows that updated the bias */
} imu_status_t;

/** Live gyro scale (starts at IMU_GYRO_SCALE; "imu,scale,<v>" changes it). */
extern float g_imu_gyro_scale;

/** Sets up I2C1 and the MPU-6050 (about 100 ms). Logs what it found.
 *  Safe to call with nothing connected. */
void imu_init(void);

/** Call often (every main-loop pass). Reads and integrates a sample when
 *  IMU_SAMPLE_US has passed. `still`: the robot is certainly not moving
 *  (stopped, wheels at rest), so the bias may be learnt now. */
void imu_poll(bool still);

/** True when the yaw is trustworthy: present, calibrated, warmed up and
 *  read within the last IMU_STALE_MS. */
bool imu_ok(void);

/** Integrated yaw, degrees, + = counter-clockwise. Only differences mean
 *  anything. */
double imu_yaw_deg(void);

/** Sets the integrated yaw to 0 (for calibrating by hand). */
void imu_zero(void);

/** Forget the bias: it is learnt again at the next still window. */
void imu_recalibrate(void);

void imu_get_status(imu_status_t *out);

#endif /* IMU_H */
