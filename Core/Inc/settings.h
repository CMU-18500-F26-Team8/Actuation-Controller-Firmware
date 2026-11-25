/**
 * @file    settings.h
 * @brief   Settings that survive power loss, kept in flash sector 7
 *          (0x08060000, the last 128 KB; the linker script keeps the
 *          firmware out of it). For now: the gyro scale from calibration.
 *
 * Each save appends one 16-byte record; the newest valid one counts. The
 * sector is erased only when it is full (every 8192 saves): that takes
 * 1-2 s, during which the CPU stalls, so save only while the robot is
 * stopped. Flashing new firmware doesn't erase the sector (only a full
 * chip erase does).
 *
 * A saved gyro scale applies only while robot_config.h still has the
 * IMU_GYRO_SCALE it was saved under: pasting a new #define and reflashing
 * overrides it.
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>

/** Scans the sector for the newest record. Call once at boot. */
void settings_init(void);

/** True if a saved gyro scale applies; *out gets it. */
bool settings_gyro_scale(float *out);

/** True if a record holds a scale that is ignored because IMU_GYRO_SCALE
 *  changed since; *saved / *saved_under get its scale and the default it
 *  was saved under. */
bool settings_gyro_scale_ignored(float *saved, float *saved_under);

/** Saves `scale` as the gyro scale (0 = forget: use IMU_GYRO_SCALE).
 *  Returns false if writing the flash failed. */
bool settings_save_gyro_scale(float scale);

#endif /* SETTINGS_H */
