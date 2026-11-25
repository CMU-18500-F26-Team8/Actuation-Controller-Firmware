/**
 * @file    drive.h
 * @brief   Differential drive: one speed PID per wheel, per-wheel odometry,
 *          a gyro heading hold on straights, and in-place turns ended and
 *          measured by the gyro (imu.h), or by the encoders without it.
 *
 * Owns the encoders and motor duties while Pi control is running: nothing
 * else may call encoder_update() or motor_set_duty() in that mode.
 * All tunables live in robot_config.h.
 */
#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DRIVE_STOPPED = 0,
  DRIVE_STRAIGHT,
  DRIVE_REVERSE,
  DRIVE_TURNING,
} drive_state_t;

typedef struct {
  float   setpoint_mps;   /* ramped setpoint the PID is tracking */
  float   speed_mps;      /* measured (windowed) */
  int16_t duty;           /* permille sent to the motor */
  float   distance_m;     /* signed, since boot */
} drive_wheel_info_t;

/** Live tunables. Start at the robot_config.h values; the USB debug port's
 *  "set,<name>,<value>" changes them until reset (see robot_app.c). */
typedef struct {
  float kp[2], ki[2], kd[2];   /* speed PID per wheel, [0] = left, [1] = right */
  float target_mps;        /* straight / reverse (TARGET_SPEED_MPS) */
  float turn_mps;          /* turning (TURN_SPEED_MPS) */
  float accel_mps2;        /* setpoint ramp (ACCEL_MPS2) */
  float hold_kp;           /* heading hold gain (HEADING_HOLD_KP), 0 = off */
} drive_tuning_t;

extern drive_tuning_t g_drive_tuning;

/** Counts control ticks (one per CONTROL_PERIOD_MS), for telemetry. */
uint32_t drive_tick_count(void);

/** Encoders must already be initialised and the driver enabled. */
void drive_init(void);

/** Runs one control tick when CONTROL_PERIOD_MS has passed. Call often. */
void drive_poll(uint32_t now_ms);

/** Straight / reverse at TARGET_SPEED_MPS. Repeating the command that is
 *  already active changes nothing (the PID is not reset). Ignored while
 *  turning; returns false then. */
bool drive_straight(void);
bool drive_reverse(void);

/** Brake now (both wheels 0 % duty, PID reset). Always accepted, also
 *  during a turn: returns true if a turn was aborted. */
bool drive_stop(void);

/** Spin in place by `deg` (positive = counter-clockwise). Ignored while a
 *  turn is already running; returns false then. */
bool drive_turn(float deg);

/** True once per finished (or stop-aborted) turn; `deg_out` gets the angle
 *  measured (gyro, or encoders without it), + = counter-clockwise. */
bool drive_take_turn_done(float *deg_out);

/** Details of the last finished turn, for the debug log. */
typedef struct {
  float    target_deg;     /* commanded */
  float    reported_deg;   /* what drive_take_turn_done gave */
  float    gyro_deg;       /* gyro's angle (meaningful if used_gyro) */
  float    enc_deg;        /* encoders' angle */
  bool     used_gyro;      /* the gyro ended and measured it */
  bool     fell_back;      /* started on the gyro, finished on the encoders */
  bool     slipped;        /* ended by the slip guard (TURN_SLIP_*) */
  bool     wrong_way;      /* gyro turned against the wheels (IMU_YAW_SIGN?) */
  bool     aborted;        /* ended by drive_stop() */
  uint32_t duration_ms;
} drive_turn_info_t;

void drive_last_turn(drive_turn_info_t *out);

/** While driving straight / backing up with the heading hold on: true, with
 *  the heading error (deg, + = robot left of its start heading) and the
 *  wheel-speed correction (m/s, + = right wheel faster). */
bool drive_hold_info(float *err_deg, float *dv_mps);

drive_state_t drive_state(void);
const char   *drive_state_name(drive_state_t s);

/** Distance travelled by the robot centre, metres, always increasing. */
float drive_odometer_m(void);
void  drive_odometer_reset(void);

void  drive_wheel_info(uint8_t left_right, drive_wheel_info_t *out);  /* 0 = L, 1 = R */

#endif /* DRIVE_H */
