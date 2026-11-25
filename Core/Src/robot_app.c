/**
 * @file    robot_app.c
 * @brief   Pi control mode: turns Pi messages into drive commands, reports
 *          odometry / turn completion / button presses, drives LD2.
 *
 * Pi -> STM32 (every line ends in '\n'):
 *   motor-instruction,straight|reverse|stop,,,
 *   motor-instruction,turn,<deg>,,        (+ = counter-clockwise, in place)
 *   odometer-reset,,,,
 *   leds-on-off,true|false,,,
 *   button-led-on-off,true|false,,,
 * STM32 -> Pi:
 *   turn-complete,<measured deg>,,,      (gyro angle; encoders without an IMU)
 *   odometer,<metres travelled>,,,       (every ODOM_PERIOD_MS while moving straight,
 *                                        once ODOM_STOP_SETTLE_MS after stopping, and
 *                                        every ODOM_IDLE_PERIOD_MS otherwise)
 *   button-press,,,,                     (B1, once per BUTTON_HOLDOFF_MS)
 *
 * IMU calibration (either port; the Pi only gets these after asking):
 *   imu[,zero|cal|scale,<v>|watch,on|off|turns,on|off]
 *   -> imu-status,<ok|no-bias|absent>,<yaw deg>,<bias deg/s>,<scale>   (Pi; USB gets a [imu] line)
 *   imu,save | imu,forget   keep the live gyro scale in flash / go back to IMU_GYRO_SCALE
 *   -> imu-saved,<ok|moving|failed>,<scale used from now on>,<flash|config>,
 *   -> turn-detail,<target>,<gyro>,<encoders>,<source>   after each turn, once the Pi sent imu,turns,on
 *
 * While a turn runs, every motor instruction except "stop" is ignored.
 * "stop" aborts the turn and turn-complete reports the angle reached.
 */
#include "robot_app.h"
#include "robot_config.h"
#include "motor_demo.h"
#include "motor.h"
#include "encoder.h"
#include "drive.h"
#include "imu.h"
#include "settings.h"
#include "pi_link.h"
#include "uart_log.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define UI_TICK_MS   10U                   /* button, LED, SF watch */

#if ROBOT_MODE != 1   /* everything below until robot_app_init is Pi mode only */

static uint32_t s_last_ui;
static uint32_t s_last_motor_cmd;
static uint32_t s_last_odom;
static bool     s_odom_final_pending;   /* final report due after a stop */
static uint32_t s_odom_stopped_at;
static uint32_t s_last_status;
static uint32_t s_last_btn_sent;
static bool     s_btn_ever_sent;
static drive_state_t s_prev_state;
static bool     s_led_alert;
static bool     s_led_button;
static bool     s_sf_prev[MOTOR_COUNT];

static bool     s_telem;                /* USB "telem,on": one T line per control tick */
static uint32_t s_telem_tick;

static uint32_t s_still_since;          /* when the robot was last seen moving */
static bool     s_imu_watch[2];         /* "imu,watch,on" per port (pi_src_t): yaw every IMU_WATCH_MS */
static uint32_t s_imu_watch_last;
static bool     s_turn_detail_pi;       /* Pi sent "imu,turns,on": turn-detail after each turn */
static const char *s_scale_source = "config";   /* where g_imu_gyro_scale came from */
#define IMU_WATCH_MS 250U

static bool     s_btn_prev_raw;
static uint8_t  s_btn_stable;
static bool     s_btn_latched;

/* ---- formatting (newlib-nano printf has no %f) ------------------------------ */

/** Writes v with `dec` decimals (0..4) into buf, e.g. "-12.5". */
static const char *fmt_fixed(char *buf, size_t n, float v, int dec, bool plus)
{
  static const long k_pow[] = { 1, 10, 100, 1000, 10000 };
  long scaled = lrintf(v * (float)k_pow[dec]);
  const char *sign = (scaled < 0) ? "-" : (plus ? "+" : "");
  long a = labs(scaled);
  if (dec == 0) snprintf(buf, n, "%s%ld", sign, a);
  else          snprintf(buf, n, "%s%ld.%0*ld", sign, a / k_pow[dec], dec, a % k_pow[dec]);
  return buf;
}

static void log_time(uint32_t now)
{
  uart_log("t=%5lu.%02lus ", (unsigned long)(now / 1000U), (unsigned long)((now % 1000U) / 10U));
}

/* ---- sending ------------------------------------------------------------------ */

static void send4(const char *type, const char *b1, const char *b2, const char *b3, const char *b4)
{
  bool ok = pi_link_send(type, b1, b2, b3, b4);
  if (DEBUG_ECHO_PI || !ok)
  {
    log_time(HAL_GetTick());
    uart_log("[tx%s] %s,%s,%s,%s,%s\n", ok ? "" : " DROPPED, buffer full", type,
             b1 ? b1 : "", b2 ? b2 : "", b3 ? b3 : "", b4 ? b4 : "");
  }
}

static void send(const char *type, const char *b1)
{
  send4(type, b1, NULL, NULL, NULL);
}

/* ---- receiving ----------------------------------------------------------------- */

static bool parse_bool(const char *s, bool *out)
{
  if (!strcasecmp(s, "true")  || !strcmp(s, "1")) { *out = true;  return true; }
  if (!strcasecmp(s, "false") || !strcmp(s, "0")) { *out = false; return true; }
  return false;
}

/* ---- live tuning (USB debug port only) ---------------------------------------
 *   get                     print the live tunables as a G line:
 *     G,<kp_l>,<ki_l>,<kd_l>,<kp_r>,<ki_r>,<kd_r>,<speed>,<turn>,<accel>
 *   set,<name>,<value>      name = kp_l ki_l kd_l kp_r ki_r kd_r speed turn accel
 *   telem,on|off            one T line per control tick (50 Hz):
 *     T,<ms>,<state>,<spL>,<vL>,<dutyL>,<spR>,<vR>,<dutyR>
 *     state 0 stopped 1 straight 2 reverse 3 turning; sp/v in mm/s; duty permille
 * Changes last until reset: copy the final values into robot_config.h. */

#define TUNING_NAMES "kp_l ki_l kd_l kp_r ki_r kd_r speed turn accel hold"

static void print_tuning(void)
{
  const drive_tuning_t *t = &g_drive_tuning;
  char v[9][16];
  const float vals[9] = { t->kp[0], t->ki[0], t->kd[0], t->kp[1], t->ki[1], t->kd[1],
                          t->target_mps, t->turn_mps, t->accel_mps2 };
  static const int dec[9] = { 2, 2, 2, 2, 2, 2, 4, 4, 3 };
  for (int i = 0; i < 9; i++) fmt_fixed(v[i], sizeof(v[i]), vals[i], dec[i], false);
  uart_log("G,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
}

/* ---- IMU calibration (USB debug port, or the Pi) ----------------------------------
 *   imu                     status: present, bias, rate, yaw, scale, counts
 *   imu,zero                yaw = 0 (then turn the robot by hand, read "imu")
 *   imu,cal                 forget the bias; relearnt at the next still moment
 *   imu,scale,<v>           live gyro scale (copy into IMU_GYRO_SCALE)
 *   imu,watch,on|off        status every 250 ms
 *   imu,turns,on|off        (Pi only) turn-detail after each turn; USB always gets [turn]
 *   imu,save                keep the live scale in flash (survives power loss)
 *   imu,forget              drop the saved scale: back to IMU_GYRO_SCALE
 * Replies go to the port that asked: a [imu] log line on USB, an imu-status
 * message to the Pi. The Pi never gets either unless it sends imu commands. */

static const char *imu_state_word(const imu_status_t *st)
{
  if (!st->present)    return "absent";
  if (!st->calibrated) return "no-bias";
  return "ok";
}

static void print_imu(void)
{
  imu_status_t st;
  imu_get_status(&st);
  char r[16], y[16], b[16], k[16];
  uart_log("[imu] %s%s WHO_AM_I 0x%02X  rate %s deg/s  yaw %s deg  bias %s deg/s  scale %s (%s)  "
           "samples %lu errors %lu bias-updates %lu\n",
           st.present ? "present" : "ABSENT", st.calibrated ? "" : " (bias not learnt yet)",
           (unsigned)st.who_am_i,
           fmt_fixed(r, sizeof(r), st.rate_dps, 2, true),
           fmt_fixed(y, sizeof(y), (float)st.yaw_deg, 2, true),
           fmt_fixed(b, sizeof(b), st.bias_dps, 3, true),
           fmt_fixed(k, sizeof(k), g_imu_gyro_scale, 4, false), s_scale_source,
           (unsigned long)st.samples, (unsigned long)st.errors, (unsigned long)st.bias_updates);
}

/** imu-status,<ok|no-bias|absent>,<yaw>,<bias>,<scale> to the Pi. */
static void send_imu_status(void)
{
  imu_status_t st;
  imu_get_status(&st);
  char y[16], b[16], k[16];
  send4("imu-status", imu_state_word(&st),
        fmt_fixed(y, sizeof(y), (float)st.yaw_deg, 2, true),
        fmt_fixed(b, sizeof(b), st.bias_dps, 3, true),
        fmt_fixed(k, sizeof(k), g_imu_gyro_scale, 4, false));
}

static void reply_imu(pi_src_t src)
{
  if (src == PI_SRC_PI) send_imu_status();
  else                  print_imu();
}

/** "on" / "off" / true / false / 1 / 0. */
static bool parse_on_off(const char *s, bool *out)
{
  if (!strcasecmp(s, "on"))  { *out = true;  return true; }
  if (!strcasecmp(s, "off")) { *out = false; return true; }
  return parse_bool(s, out);
}

static void handle_imu(const pi_msg_t *m)
{
  const char *sub = m->field[1];
  pi_src_t src = m->src;
  bool on;

  if (sub[0] == '\0') { reply_imu(src); return; }
  if (!strcasecmp(sub, "zero")) { imu_zero(); reply_imu(src); return; }
  if (!strcasecmp(sub, "cal"))
  {
    imu_recalibrate();
    uart_log("[imu] bias forgotten; relearnt after %u ms standing still\n", (unsigned)IMU_STILL_AFTER_MS);
    reply_imu(src);
    return;
  }
  if (!strcasecmp(sub, "watch"))
  {
    if (parse_on_off(m->field[2], &on))
    {
      s_imu_watch[src] = on;
      uart_log("[imu] watch %s (%s)\n", on ? "on" : "off", (src == PI_SRC_PI) ? "to the Pi" : "here");
    }
    else uart_log("      ignored: imu,watch,on or imu,watch,off\n");
    reply_imu(src);
    return;
  }
  if (!strcasecmp(sub, "turns"))
  {
    if (src != PI_SRC_PI)
      uart_log("      [turn] lines always print here; imu,turns is for the Pi\n");
    else if (parse_on_off(m->field[2], &on))
    {
      s_turn_detail_pi = on;
      uart_log("[imu] turn-detail to the Pi %s\n", on ? "on" : "off");
    }
    else uart_log("      ignored: imu,turns,on or imu,turns,off\n");
    reply_imu(src);
    return;
  }
  if (!strcasecmp(sub, "scale"))
  {
    char *end = NULL;
    float v = strtof(m->field[2], &end);
    if (m->field[2][0] == '\0' || !end || *end != '\0' || !isfinite(v) || v < 0.5f || v > 1.5f)
      uart_log("      ignored: imu,scale needs a number from 0.5 to 1.5\n");
    else
    {
      g_imu_gyro_scale = v;
      s_scale_source = "live, not saved";
    }
    reply_imu(src);
    return;
  }
  if (!strcasecmp(sub, "save") || !strcasecmp(sub, "forget"))
  {
    bool forget = !strcasecmp(sub, "forget");
    const char *result;
    if (drive_state() != DRIVE_STOPPED)
      result = "moving";                   /* a full-sector erase stalls the CPU 1-2 s */
    else if (!settings_save_gyro_scale(forget ? 0.0f : g_imu_gyro_scale))
      result = "failed";
    else
    {
      result = "ok";
      if (forget) g_imu_gyro_scale = IMU_GYRO_SCALE;
      s_scale_source = forget ? "config" : "flash";
    }
    char k[16];
    fmt_fixed(k, sizeof(k), g_imu_gyro_scale, 4, false);
    uart_log("[imu] %s: %s (scale %s, %s)\n", forget ? "forget" : "save",
             !strcmp(result, "ok") ? "done" : !strcmp(result, "moving") ? "refused, the robot is moving"
                                                                       : "FLASH WRITE FAILED",
             k, s_scale_source);
    if (src == PI_SRC_PI)
      send4("imu-saved", result, k, !strcmp(s_scale_source, "flash") ? "flash" : "config", NULL);
    return;
  }
  uart_log("      ignored: imu | imu,zero | imu,cal | imu,scale,<v> | imu,save | imu,forget | "
           "imu,watch,on|off | imu,turns,on|off\n");
  reply_imu(src);
}

/** Returns true if the message was a tuning command (handled here). */
static bool handle_tuning(const pi_msg_t *m)
{
  const char *type = m->field[0];
  if (!strcasecmp(type, "get"))
  {
    char h[12];
    print_tuning();                        /* (the G line stays 9 fields for pid_tune.py) */
    uart_log("hold %s (heading hold KP, 1/s; 0 = off)\n", fmt_fixed(h, sizeof(h), g_drive_tuning.hold_kp, 2, false));
    return true;
  }
  if (!strcasecmp(type, "telem"))
  {
    bool on;
    if (parse_bool(m->field[1], &on) || !strcasecmp(m->field[1], "on") || !strcasecmp(m->field[1], "off"))
    {
      s_telem = !strcasecmp(m->field[1], "on") || (parse_bool(m->field[1], &on) && on);
      s_telem_tick = drive_tick_count();
      uart_log("[telem] %s\n", s_telem ? "on" : "off");
    }
    else uart_log("      ignored: telem,on or telem,off\n");
    return true;
  }
  if (!strcasecmp(type, "set"))
  {
    static const struct { const char *name; float *v; float lo, hi; } k_vars[] = {
      { "kp_l",  &g_drive_tuning.kp[0],      0.0f, 50000.0f },
      { "ki_l",  &g_drive_tuning.ki[0],      0.0f, 500000.0f },
      { "kd_l",  &g_drive_tuning.kd[0],      0.0f, 5000.0f },
      { "kp_r",  &g_drive_tuning.kp[1],      0.0f, 50000.0f },
      { "ki_r",  &g_drive_tuning.ki[1],      0.0f, 500000.0f },
      { "kd_r",  &g_drive_tuning.kd[1],      0.0f, 5000.0f },
      { "speed", &g_drive_tuning.target_mps, 0.0f, 0.6f },
      { "turn",  &g_drive_tuning.turn_mps,   TURN_MIN_SPEED_MPS, 0.6f },
      { "accel", &g_drive_tuning.accel_mps2, 0.01f, 100.0f },
      { "hold",  &g_drive_tuning.hold_kp,    0.0f, 20.0f },
    };
    char *end = NULL;
    float v = strtof(m->field[2], &end);
    bool num_ok = m->field[2][0] != '\0' && end && *end == '\0' && isfinite(v);
    for (size_t i = 0; i < sizeof(k_vars) / sizeof(k_vars[0]); i++)
    {
      if (strcasecmp(m->field[1], k_vars[i].name)) continue;
      if (!num_ok || v < k_vars[i].lo || v > k_vars[i].hi)
      {
        char lo[16], hi[16];
        uart_log("      ignored: %s must be a number from %s to %s\n", k_vars[i].name,
                 fmt_fixed(lo, sizeof(lo), k_vars[i].lo, 2, false), fmt_fixed(hi, sizeof(hi), k_vars[i].hi, 2, false));
        return true;
      }
      *k_vars[i].v = v;
      print_tuning();
      if (k_vars[i].v == &g_drive_tuning.hold_kp)
      {
        char h[12];
        uart_log("hold %s (heading hold KP, 1/s; 0 = off)\n", fmt_fixed(h, sizeof(h), v, 2, false));
      }
      return true;
    }
    uart_log("      ignored: unknown name \"%s\" (" TUNING_NAMES ")\n", m->field[1]);
    return true;
  }
  return false;
}

static void handle(const pi_msg_t *m, uint32_t now)
{
  const char *type = m->field[0];
  const char *b1   = m->field[1];

  if (DEBUG_ECHO_PI || m->src == PI_SRC_DEBUG)
  {
    log_time(now);
    uart_log("[rx %s] %s,%s,%s,%s,%s\n", (m->src == PI_SRC_PI) ? "pi " : "usb",
             m->field[0], m->field[1], m->field[2], m->field[3], m->field[4]);
  }

  if (!strcasecmp(type, "imu")) { handle_imu(m); return; }   /* either port */
  if (m->src == PI_SRC_DEBUG && handle_tuning(m)) return;

  if (!strcasecmp(type, "motor-instruction"))
  {
    if (!strcasecmp(b1, "stop"))
    {
      s_last_motor_cmd = now;
      if (drive_stop()) uart_log("      stop: turn aborted\n");
      return;
    }
    if (drive_state() == DRIVE_TURNING)
    {
      uart_log("      ignored: turn in progress\n");
      return;
    }
    if (!strcasecmp(b1, "straight"))      { s_last_motor_cmd = now; (void)drive_straight(); }
    else if (!strcasecmp(b1, "reverse"))  { s_last_motor_cmd = now; (void)drive_reverse(); }
    else if (!strcasecmp(b1, "turn"))
    {
      char *end = NULL;
      float deg = strtof(m->field[2], &end);
      if (m->field[2][0] == '\0' || end == NULL || *end != '\0' || !isfinite(deg))
      {
        uart_log("      ignored: bad turn angle \"%s\"\n", m->field[2]);
        return;
      }
      s_last_motor_cmd = now;
      (void)drive_turn(deg);
    }
    else uart_log("      ignored: unknown motor instruction \"%s\"\n", b1);
    return;
  }

  if (!strcasecmp(type, "odometer-reset"))
  {
    drive_odometer_reset();
    return;
  }

  if (!strcasecmp(type, "leds-on-off") || !strcasecmp(type, "button-led-on-off"))
  {
    bool on;
    if (!parse_bool(b1, &on))
    {
      uart_log("      ignored: expected true/false, got \"%s\"\n", b1);
      return;
    }
    if (type[0] == 'l' || type[0] == 'L') s_led_alert = on;
    else                                  s_led_button = on;
    return;
  }

  uart_log("      ignored: unknown message type \"%s\"\n", type);
}

/* ---- B1 (front button), LD2, SF watch ------------------------------------------- */

/** Debounced press edge on B1 (active low, 3 stable 10 ms ticks). */
static bool button_pressed_edge(void)
{
  bool raw = (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET);
  if (raw == s_btn_prev_raw) { if (s_btn_stable < 255U) s_btn_stable++; }
  else                       { s_btn_stable = 0U; s_btn_prev_raw = raw; }
  if (s_btn_stable == 3U)
  {
    if (raw && !s_btn_latched) { s_btn_latched = true; return true; }
    if (!raw)                    s_btn_latched = false;
  }
  return false;
}

/** LD2 is the only user LED: alert = solid, button LED only = 1 Hz blink. */
static void update_led(uint32_t now)
{
  bool on = s_led_alert || (s_led_button && ((now / 500U) % 2U) == 0U);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ui_tick(uint32_t now)
{
  if (button_pressed_edge())
  {
    if (!s_btn_ever_sent || (now - s_last_btn_sent) >= BUTTON_HOLDOFF_MS)
    {
      s_btn_ever_sent = true;
      s_last_btn_sent = now;
      send("button-press", NULL);
    }
  }
  update_led(now);

  /* Faults are not acted on yet; just log SF changes for diagnosis. */
  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    bool low = motor_sf_pin_low(m);
    if (low != s_sf_prev[m])
    {
      s_sf_prev[m] = low;
      log_time(now);
      uart_log("[sf] M%uSF %s\n", (unsigned)(m + 1U), low ? "LOW (driver reports fault)" : "back HIGH");
    }
  }
}

static void status_line(uint32_t now)
{
  char a[12], b[12], c[12], o[12];
  log_time(now);
  uart_log("%-8s", drive_state_name(drive_state()));
  for (uint8_t i = 0; i < 2; i++)
  {
    drive_wheel_info_t w;
    drive_wheel_info(i, &w);
    uart_log(" %s sp%s v%s d%s%%", i ? "| R" : "L",
             fmt_fixed(a, sizeof(a), w.setpoint_mps, 3, true),
             fmt_fixed(b, sizeof(b), w.speed_mps, 3, true),
             fmt_fixed(c, sizeof(c), (float)w.duty / 10.0f, 1, true));
  }
  uart_log(" | odom %s m", fmt_fixed(o, sizeof(o), drive_odometer_m(), 3, false));
  if (imu_ok()) uart_log(" | yaw %s", fmt_fixed(o, sizeof(o), (float)imu_yaw_deg(), 1, true));
  float herr, hdv;
  if (drive_hold_info(&herr, &hdv))
  {
    char e[12], v[12];
    uart_log(" | hold err %s dv %s", fmt_fixed(e, sizeof(e), herr, 2, true), fmt_fixed(v, sizeof(v), hdv, 3, true));
  }
  uart_log("\n");
}

/* ---- boot check ------------------------------------------------------------------ */

/** Same two-stage check as the demo's self-test, logged only (faults are not
 *  acted on yet). Leaves the driver enabled and braking. */
static void boot_selftest(void)
{
  motor_wake();                            /* EN HIGH, D2 LOW: SF must be LOW */
  HAL_Delay(20);
  bool a1 = motor_sf_pin_low(MOTOR_1), a2 = motor_sf_pin_low(MOTOR_2);
  motor_enable(true);                      /* D2 HIGH, 0 %: SF must be HIGH */
  HAL_Delay(20);
  bool b1 = motor_sf_pin_low(MOTOR_1), b2 = motor_sf_pin_low(MOTOR_2);
  bool ok = a1 && a2 && !b1 && !b2;
  uart_log("[self-test] D2 LOW: M1SF=%s M2SF=%s (expect LOW)  D2 HIGH: M1SF=%s M2SF=%s (expect HIGH)  %s\n",
           a1 ? "LOW" : "HIGH", a2 ? "LOW" : "HIGH", b1 ? "LOW" : "HIGH", b2 ? "LOW" : "HIGH",
           ok ? "pass" : "FAIL (check driver wiring / battery)");
  s_sf_prev[MOTOR_1] = b1;
  s_sf_prev[MOTOR_2] = b2;
}

/* ---------------------------------------------------------------------------------- */

#endif /* ROBOT_MODE != 1 */

void robot_app_init(void)
{
#if ROBOT_MODE == 1
  motor_demo_init();
#else
  uart_log_init();
  motor_init();
  encoder_init();
  pi_link_init();

  uart_log("\n\n=== SnackBot motor controller: Pi control mode ===\n");
  uart_log("Pi link: UART4 %u baud, A0 = TX -> Pi RXD, A1 = RX <- Pi TXD, lines end in \\n\n",
           (unsigned)PI_UART_BAUD);
  uart_log("Commands can also be typed here (end with Enter). B1 = front button, LD2 = LEDs.\n");
  uart_log("Tuning (here only): get | set,<name>,<value> | telem,on|off   names: " TUNING_NAMES "\n");
  uart_log("IMU (here or from the Pi): imu | imu,zero | imu,cal | imu,scale,<v> | imu,save | imu,forget | "
           "imu,watch,on|off | imu,turns,on|off\n");
  {
    char s1[12], s2[12], s3[12], s4[12];
    uart_log("Target %s m/s, turns %s m/s, wheel %s m, track %s m\n",
             fmt_fixed(s1, sizeof(s1), TARGET_SPEED_MPS, 3, false),
             fmt_fixed(s2, sizeof(s2), TURN_SPEED_MPS, 3, false),
             fmt_fixed(s3, sizeof(s3), WHEEL_DIAMETER_M, 4, false),
             fmt_fixed(s4, sizeof(s4), TRACK_WIDTH_M, 4, false));
  }
  boot_selftest();
  settings_init();
  {
    float v, under;
    char a[16], b[16];
    if (settings_gyro_scale(&v))
    {
      g_imu_gyro_scale = v;
      s_scale_source = "flash";
      uart_log("[imu] gyro scale %s, saved in flash\n", fmt_fixed(a, sizeof(a), v, 4, false));
    }
    else if (settings_gyro_scale_ignored(&v, &under))
    {
      fmt_fixed(a, sizeof(a), v, 4, false);
      fmt_fixed(b, sizeof(b), under, 4, false);
      uart_log("[imu] saved gyro scale %s ignored: saved when IMU_GYRO_SCALE was %s, robot_config.h changed it\n", a, b);
    }
  }
  imu_init();
  drive_init();

  uint32_t now = HAL_GetTick();
  s_last_ui = s_last_motor_cmd = s_last_odom = s_last_status = now;
  s_still_since = now;
  s_prev_state = DRIVE_STOPPED;
  s_led_alert = s_led_button = false;
  update_led(now);
  uart_log("[ready] stopped, waiting for the Pi\n");
#endif
}

void robot_app_poll(void)
{
#if ROBOT_MODE == 1
  motor_demo_poll();
#else
  uint32_t now = HAL_GetTick();
  pi_msg_t msg;

  while (pi_link_get(&msg)) handle(&msg, now);

  /* Still = stopped and both wheels at rest for IMU_STILL_AFTER_MS: the gyro
   * may learn its bias. */
  {
    drive_wheel_info_t l, r;
    drive_wheel_info(0, &l);
    drive_wheel_info(1, &r);
    if (drive_state() != DRIVE_STOPPED || l.speed_mps != 0.0f || r.speed_mps != 0.0f)
      s_still_since = now;
  }
  imu_poll((now - s_still_since) >= IMU_STILL_AFTER_MS);

  drive_poll(now);

  float deg;
  if (drive_take_turn_done(&deg))
  {
    char b[12];
    send("turn-complete", fmt_fixed(b, sizeof(b), deg, 1, false));

    drive_turn_info_t ti;
    char t[12], g[12], e[12];
    drive_last_turn(&ti);
    log_time(now);
    uart_log("[turn] target %s  gyro %s  encoders %s  -> reported %s (%s%s%s), %lu ms\n",
             fmt_fixed(t, sizeof(t), ti.target_deg, 1, true),
             ti.used_gyro ? fmt_fixed(g, sizeof(g), ti.gyro_deg, 1, true) : "n/a",
             fmt_fixed(e, sizeof(e), ti.enc_deg, 1, true), b,
             ti.used_gyro ? "gyro" : "encoders",
             ti.wrong_way ? ", GYRO TURNED THE WRONG WAY: check IMU_YAW_SIGN"
                          : (ti.fell_back ? ", gyro lost mid-turn" : ""),
             ti.slipped ? ", SLIP: wheels far ahead of the gyro, stopped early"
                        : (ti.aborted ? ", aborted by stop" : ""),
             (unsigned long)ti.duration_ms);

    if (s_turn_detail_pi)
    {
      const char *srcw = ti.slipped ? "gyro-slip"
                       : ti.wrong_way ? "encoders-wrong-way"
                       : ti.fell_back ? "encoders-gyro-lost"
                       : ti.used_gyro ? "gyro" : "encoders";
      send4("turn-detail", t, ti.used_gyro ? g : "", e, srcw);
    }
  }

  drive_state_t st = drive_state();
  bool moving = (st == DRIVE_STRAIGHT || st == DRIVE_REVERSE);

  if (moving && CMD_WATCHDOG_MS > 0U && (now - s_last_motor_cmd) >= CMD_WATCHDOG_MS)
  {
    (void)drive_stop();
    log_time(now);
    uart_log("[watchdog] no motor instruction for %u ms: stopped\n", (unsigned)CMD_WATCHDOG_MS);
    st = drive_state();
    moving = false;
  }

  bool was_moving = (s_prev_state == DRIVE_STRAIGHT || s_prev_state == DRIVE_REVERSE);
  if (moving)
  {
    s_odom_final_pending = false;          /* moving again: periodic reports resume */
    if (!was_moving) s_last_odom = now;    /* first report one period after starting */
    if ((now - s_last_odom) >= ODOM_PERIOD_MS)
    {
      char b[12];
      s_last_odom = now;
      send("odometer", fmt_fixed(b, sizeof(b), drive_odometer_m(), 3, false));
    }
  }
  else if (was_moving && ODOM_STOP_SETTLE_MS > 0U)
  {
    s_odom_final_pending = true;           /* just stopped (or a turn began) */
    s_odom_stopped_at = now;
  }
  if (s_odom_final_pending && (now - s_odom_stopped_at) >= ODOM_STOP_SETTLE_MS)
  {
    char b[12];
    s_odom_final_pending = false;
    s_last_odom = now;
    send("odometer", fmt_fixed(b, sizeof(b), drive_odometer_m(), 3, false));
  }
  else if (!moving && !s_odom_final_pending && ODOM_IDLE_PERIOD_MS > 0U
           && (now - s_last_odom) >= ODOM_IDLE_PERIOD_MS)
  {
    char b[12];                            /* stopped or turning: slow heartbeat */
    s_last_odom = now;
    send("odometer", fmt_fixed(b, sizeof(b), drive_odometer_m(), 3, false));
  }

  if (s_telem && drive_tick_count() != s_telem_tick)
  {
    s_telem_tick = drive_tick_count();
    drive_wheel_info_t l, r;
    drive_wheel_info(0, &l);
    drive_wheel_info(1, &r);
    uart_log("T,%lu,%u,%ld,%ld,%d,%ld,%ld,%d\n", (unsigned long)now, (unsigned)st,
             lrintf(l.setpoint_mps * 1000.0f), lrintf(l.speed_mps * 1000.0f), (int)l.duty,
             lrintf(r.setpoint_mps * 1000.0f), lrintf(r.speed_mps * 1000.0f), (int)r.duty);
  }

  if ((s_imu_watch[PI_SRC_DEBUG] || s_imu_watch[PI_SRC_PI]) && (now - s_imu_watch_last) >= IMU_WATCH_MS)
  {
    s_imu_watch_last = now;
    if (s_imu_watch[PI_SRC_DEBUG])
    {
      imu_status_t is;
      char y[16], r[16];
      imu_get_status(&is);
      log_time(now);
      uart_log("[imu] yaw %s deg  rate %s deg/s%s\n",
               fmt_fixed(y, sizeof(y), (float)is.yaw_deg, 2, true),
               fmt_fixed(r, sizeof(r), is.rate_dps, 2, true),
               imu_ok() ? "" : (is.present ? "  (bias not learnt yet)" : "  (IMU absent)"));
    }
    if (s_imu_watch[PI_SRC_PI]) send_imu_status();
  }

  if (!s_telem && DEBUG_STATUS_MS > 0U && st != DRIVE_STOPPED && (now - s_last_status) >= DEBUG_STATUS_MS)
  {
    s_last_status = now;
    status_line(now);
  }
  if (st != s_prev_state)
  {
    log_time(now);
    uart_log("[state] %s -> %s\n", drive_state_name(s_prev_state), drive_state_name(st));
    s_prev_state = st;
  }

  if ((now - s_last_ui) >= UI_TICK_MS)
  {
    s_last_ui = now;
    ui_tick(now);
  }
#endif
}
