/**
 * @file    drive.c
 * @brief   One speed PID per wheel, odometry, in-place turns.
 *
 * Every CONTROL_PERIOD_MS:
 *   1. read both encoders, update per-wheel position, windowed speed and
 *      the robot odometer (|left + right| / 2, so spins add ~0)
 *   2. pick each wheel's signed target speed from the drive state
 *      (straight / reverse / turning with a slow-down near the end)
 *      A turn ends when the gyro has turned the angle (imu.h), or, without a
 *      trustworthy gyro, when the wheels have travelled its arc. A gyro turn
 *      brakes, waits TURN_SETTLE_MS and reports the gyro's angle.
 *   3. ramp each setpoint by at most ACCEL_MPS2
 *   4. one PID per wheel (its own gains) -> motor duty
 */
#include "drive.h"
#include "robot_config.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdlib.h>

#define PI_F             3.14159265f
#define M_PER_COUNT      ((PI_F * WHEEL_DIAMETER_M) / (float)ENC_COUNTS_PER_OUTPUT_REV)
#define HALF_TRACK_M     (0.5f * TRACK_WIDTH_M)
#define DEG_TO_RAD       (PI_F / 180.0f)

typedef struct {
  uint8_t  motor;                          /* MOTOR_1 / MOTOR_2, also encoder index */
  int32_t  pos;                            /* counts since boot, signed */
  int32_t  pos0;                           /* pos when the current command started */
  int32_t  win_counts[SPEED_WINDOW_TICKS];
  uint32_t win_ms[SPEED_WINDOW_TICKS];
  float    speed;                          /* m/s, measured */
  float    sp;                             /* m/s, ramped setpoint */
  float    integ;                          /* integral of error, m */
  float    prev_err;
  bool     have_prev;
  int16_t  duty;
} wheel_t;

static wheel_t       s_w[2];               /* 0 = left, 1 = right */
static uint8_t       s_win_idx;
static drive_state_t s_state;
static float         s_dir[2];             /* commanded direction per wheel: +1 / -1 */
static float         s_turn_arc_m;         /* wheel travel needed for the turn */
static float         s_turn_sign;          /* +1 = CCW */
static float         s_turn_target_deg;    /* |commanded angle| */
static bool          s_turn_gyro;          /* the gyro is ending this turn */
static bool          s_turn_fell_back;     /* it started on the gyro but lost it */
static bool          s_turn_slipped;
static bool          s_turn_wrong_way;     /* gyro turned against the wheels */
static bool          s_turn_settling;      /* braked, waiting TURN_SETTLE_MS */
static uint32_t      s_turn_settle_start;
static uint32_t      s_turn_start_ms;
static double        s_turn_yaw0;          /* gyro yaw when the turn started */
static bool          s_turn_done;
static float         s_turn_done_deg;
static drive_turn_info_t s_last_turn;
static uint32_t      s_odom_halfcounts;    /* sum of |dL + dR| */
static uint32_t      s_last_tick;
static uint32_t      s_tick_count;

drive_tuning_t g_drive_tuning = {
  .kp = { PID_L_KP, PID_R_KP }, .ki = { PID_L_KI, PID_R_KI }, .kd = { PID_L_KD, PID_R_KD },
  .target_mps = TARGET_SPEED_MPS, .turn_mps = TURN_SPEED_MPS,
  .accel_mps2 = ACCEL_MPS2,
};
#define T g_drive_tuning

/* ---------------------------------------------------------------------------- */

static void pid_reset(wheel_t *w)
{
  w->integ = 0.0f;
  w->prev_err = 0.0f;
  w->have_prev = false;
}

static void brake_all(void)
{
  for (uint8_t i = 0; i < 2; i++)
  {
    s_w[i].duty = 0;
    s_w[i].sp = 0.0f;
    pid_reset(&s_w[i]);
    motor_set_duty(s_w[i].motor, 0);
  }
}

/** Signed progress (m) of wheel i along its commanded direction. */
static float progress(uint8_t i)
{
  return s_dir[i] * (float)(s_w[i].pos - s_w[i].pos0) * M_PER_COUNT;
}

/** Angle (deg, + = CCW) turned since the command started, from the encoders. */
static float measured_turn_deg(void)
{
  float dl = (float)(s_w[0].pos - s_w[0].pos0) * M_PER_COUNT;
  float dr = (float)(s_w[1].pos - s_w[1].pos0) * M_PER_COUNT;
  return ((dr - dl) / TRACK_WIDTH_M) / DEG_TO_RAD;
}

static void begin_command(drive_state_t st, float dir_l, float dir_r)
{
  s_state = st;
  s_dir[0] = dir_l;
  s_dir[1] = dir_r;
  for (uint8_t i = 0; i < 2; i++)
  {
    s_w[i].pos0 = s_w[i].pos;
    pid_reset(&s_w[i]);                    /* setpoint stays continuous */
  }
}

/** Angle (deg, + = CCW) the gyro has turned since the turn started. */
static float gyro_turn_deg(void)
{
  return (float)(imu_yaw_deg() - s_turn_yaw0);
}

static void finish_turn(bool aborted)
{
  if (s_turn_gyro && !imu_ok())            /* lost it while settling */
  {
    s_turn_gyro = false;
    s_turn_fell_back = true;
  }
  float enc = measured_turn_deg();
  float gyr = s_turn_gyro ? gyro_turn_deg() : 0.0f;
  s_turn_done_deg = s_turn_gyro ? gyr : enc;
  s_turn_done = true;
  s_turn_settling = false;

  s_last_turn.target_deg   = s_turn_sign * s_turn_target_deg;
  s_last_turn.reported_deg = s_turn_done_deg;
  s_last_turn.gyro_deg     = gyr;
  s_last_turn.enc_deg      = enc;
  s_last_turn.used_gyro    = s_turn_gyro;
  s_last_turn.fell_back    = s_turn_fell_back;
  s_last_turn.slipped      = s_turn_slipped;
  s_last_turn.wrong_way    = s_turn_wrong_way;
  s_last_turn.aborted      = aborted;
  s_last_turn.duration_ms  = HAL_GetTick() - s_turn_start_ms;
}

/** Brake and end the turn: gyro turns settle first, encoder turns end now. */
static void end_turn(uint32_t now)
{
  brake_all();
  if (s_turn_gyro && TURN_SETTLE_MS > 0U)
  {
    s_turn_settling = true;
    s_turn_settle_start = now;
    return;
  }
  finish_turn(false);
  s_state = DRIVE_STOPPED;
}

static float clampf(float v, float lo, float hi)
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static float ramp_toward(float cur, float target, float step)
{
  if (target > cur + step) return cur + step;
  if (target < cur - step) return cur - step;
  return target;
}

static int16_t pid_step(wheel_t *w, uint8_t i, float dt)
{
  float err   = w->sp - w->speed;
  float deriv = w->have_prev ? (err - w->prev_err) / dt : 0.0f;
  w->prev_err  = err;
  w->have_prev = true;

  float base = T.kp[i] * err + T.kd[i] * deriv;

  /* Conditional integration (anti-windup): only integrate when the output
   * is not saturated, or when this error would pull it back in. */
  float trial = base + T.ki[i] * (w->integ + err * dt);
  if (fabsf(trial) <= (float)DUTY_LIMIT || (trial > 0.0f) != (err > 0.0f))
    w->integ += err * dt;

  float u = clampf(base + T.ki[i] * w->integ, -(float)DUTY_LIMIT, (float)DUTY_LIMIT);
  return (int16_t)lrintf(u);
}

/* ---------------------------------------------------------------------------- */

void drive_init(void)
{
  s_w[0].motor = LEFT_MOTOR;
  s_w[1].motor = RIGHT_MOTOR;
  for (uint8_t i = 0; i < 2; i++)
  {
    s_w[i].pos = 0;
    s_w[i].pos0 = 0;
    s_w[i].speed = 0.0f;
    for (uint8_t k = 0; k < SPEED_WINDOW_TICKS; k++)
    {
      s_w[i].win_counts[k] = 0;
      s_w[i].win_ms[k] = 0U;
    }
    (void)encoder_update(s_w[i].motor);    /* discard counts from before */
  }
  s_win_idx = 0U;
  s_dir[0] = s_dir[1] = 1.0f;
  s_state = DRIVE_STOPPED;
  s_turn_done = false;
  s_turn_settling = false;
  s_odom_halfcounts = 0U;
  s_last_tick = HAL_GetTick();
  brake_all();
}

void drive_poll(uint32_t now)
{
  uint32_t dt_ms = now - s_last_tick;
  if (dt_ms < CONTROL_PERIOD_MS) return;
  s_last_tick = now;
  float dt = (float)dt_ms * 0.001f;

  /* ---- 1. measure ---------------------------------------------------------- */
  int32_t d[2];
  for (uint8_t i = 0; i < 2; i++)
  {
    wheel_t *w = &s_w[i];
    d[i] = encoder_update(w->motor);
    w->pos += d[i];
    w->win_counts[s_win_idx] = d[i];
    w->win_ms[s_win_idx] = dt_ms;

    int32_t  sum_c = 0;
    uint32_t sum_ms = 0U;
    for (uint8_t k = 0; k < SPEED_WINDOW_TICKS; k++)
    {
      sum_c  += w->win_counts[k];
      sum_ms += w->win_ms[k];
    }
    w->speed = (sum_ms > 0U) ? ((float)sum_c * M_PER_COUNT) / ((float)sum_ms * 0.001f) : 0.0f;
  }
  s_win_idx = (uint8_t)((s_win_idx + 1U) % SPEED_WINDOW_TICKS);
  s_odom_halfcounts += (uint32_t)labs((long)(d[0] + d[1]));
  s_tick_count++;

  if (s_state == DRIVE_STOPPED) return;    /* braking; duties already 0 */

  /* ---- 2. target speed magnitude ------------------------------------------- */
  float mag;
  if (s_state == DRIVE_TURNING)
  {
    if (s_turn_settling)                   /* braked; duties already 0 */
    {
      if ((now - s_turn_settle_start) >= TURN_SETTLE_MS)
      {
        finish_turn(false);
        s_state = DRIVE_STOPPED;
      }
      return;
    }
    if (s_turn_gyro && !imu_ok())          /* gyro lost: encoders finish it */
    {
      s_turn_gyro = false;
      s_turn_fell_back = true;
    }

    /* Degrees still to go, along the commanded direction. */
    float enc_deg = (0.5f * (progress(0) + progress(1))) / HALF_TRACK_M / DEG_TO_RAD;
    if (s_turn_gyro && enc_deg > TURN_WRONG_WAY_DEG
        && s_turn_sign * gyro_turn_deg() < -0.5f * enc_deg)
    {
      s_turn_gyro = false;                 /* IMU_YAW_SIGN wrong: encoders finish it */
      s_turn_fell_back = true;
      s_turn_wrong_way = true;
    }
    float remaining = s_turn_target_deg - (s_turn_gyro ? s_turn_sign * gyro_turn_deg() : enc_deg);
    if (remaining <= 0.0f)
    {
      end_turn(now);
      return;
    }
    if (s_turn_gyro && enc_deg > TURN_SLIP_RATIO * s_turn_target_deg + TURN_SLIP_EXTRA_DEG)
    {
      s_turn_slipped = true;               /* wheels far ahead of the gyro */
      end_turn(now);
      return;
    }
    mag = (remaining >= TURN_SLOW_DEG)
            ? T.turn_mps
            : TURN_MIN_SPEED_MPS + (T.turn_mps - TURN_MIN_SPEED_MPS) * (remaining / TURN_SLOW_DEG);
  }
  else
  {
    mag = T.target_mps;
  }

  /* ---- 3 + 4. ramp, PID, output ------------------------------------------------ */
  for (uint8_t i = 0; i < 2; i++)
  {
    wheel_t *w = &s_w[i];
    w->sp = ramp_toward(w->sp, s_dir[i] * mag, T.accel_mps2 * dt);
    w->duty = pid_step(w, i, dt);
    motor_set_duty(w->motor, w->duty);
  }
}

bool drive_straight(void)
{
  if (s_state == DRIVE_TURNING) return false;
  if (s_state != DRIVE_STRAIGHT) begin_command(DRIVE_STRAIGHT, 1.0f, 1.0f);
  return true;
}

bool drive_reverse(void)
{
  if (s_state == DRIVE_TURNING) return false;
  if (s_state != DRIVE_REVERSE) begin_command(DRIVE_REVERSE, -1.0f, -1.0f);
  return true;
}

bool drive_stop(void)
{
  bool aborted = (s_state == DRIVE_TURNING);
  if (aborted) finish_turn(true);
  s_state = DRIVE_STOPPED;
  brake_all();
  return aborted;
}

bool drive_turn(float deg)
{
  if (s_state == DRIVE_TURNING) return false;
  s_turn_sign       = (deg >= 0.0f) ? 1.0f : -1.0f;
  s_turn_target_deg = fabsf(deg);
  s_turn_arc_m      = s_turn_target_deg * DEG_TO_RAD * HALF_TRACK_M;
  s_turn_gyro       = IMU_TURNS && imu_ok();
  s_turn_yaw0       = imu_yaw_deg();
  s_turn_fell_back  = false;
  s_turn_slipped    = false;
  s_turn_wrong_way  = false;
  s_turn_settling   = false;
  s_turn_start_ms   = HAL_GetTick();
  /* CCW (+): left wheel backwards, right wheel forwards. */
  begin_command(DRIVE_TURNING, -s_turn_sign, s_turn_sign);
  if (s_turn_arc_m < M_PER_COUNT)          /* nothing to do (e.g. 0 deg) */
  {
    finish_turn(false);
    s_state = DRIVE_STOPPED;
    brake_all();
  }
  return true;
}

bool drive_take_turn_done(float *deg_out)
{
  if (!s_turn_done) return false;
  s_turn_done = false;
  if (deg_out) *deg_out = s_turn_done_deg;
  return true;
}

void drive_last_turn(drive_turn_info_t *out) { *out = s_last_turn; }

drive_state_t drive_state(void) { return s_state; }

uint32_t drive_tick_count(void) { return s_tick_count; }

const char *drive_state_name(drive_state_t s)
{
  switch (s)
  {
    case DRIVE_STRAIGHT: return "straight";
    case DRIVE_REVERSE:  return "reverse";
    case DRIVE_TURNING:  return "turning";
    default:             return "stopped";
  }
}

float drive_odometer_m(void)
{
  return (float)s_odom_halfcounts * 0.5f * M_PER_COUNT;
}

void drive_odometer_reset(void)
{
  s_odom_halfcounts = 0U;
}

void drive_wheel_info(uint8_t lr, drive_wheel_info_t *out)
{
  const wheel_t *w = &s_w[lr ? 1 : 0];
  out->setpoint_mps = w->sp;
  out->speed_mps    = w->speed;
  out->duty         = w->duty;
  out->distance_m   = (float)w->pos * M_PER_COUNT;
}
