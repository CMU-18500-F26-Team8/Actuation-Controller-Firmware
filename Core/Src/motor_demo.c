/**
 * @file    motor_demo.c
 * @brief   Open-loop two-motor demo: same direction, opposite directions,
 *          both ways, with encoder telemetry and fault handling.
 */
#include "motor_demo.h"
#include "motor_pins.h"
#include "motor.h"
#include "encoder.h"
#include "uart_log.h"
#include "main.h"

#include <stdbool.h>
#include <stdlib.h>

#define TICK_MS            10U    /* profile + button rate */
#define TELEMETRY_MS       100U   /* speed estimate + print rate */
#define SELFTEST_STEP_MS   20U    /* per self-test stage; MC33926 wake-up is <= 5 ms */

#define RAMP_MS            250U   /* short soft start/stop, spares the gearbox */
#define HOLD_MS            (DEMO_SPIN ? DEMO_SPIN_HOLD_MS : DEMO_HOLD_MS)
#define PAUSE_MS           500U
#define MOVE_MS            (RAMP_MS + HOLD_MS + RAMP_MS)
#define PHASE_MS           (MOVE_MS + PAUSE_MS)

#define USE_M1             (DEMO_ONLY_MOTOR != 2)
#define USE_M2             (DEMO_ONLY_MOTOR != 1)
#define MOTORS_LABEL       ((DEMO_ONLY_MOTOR == 1) ? "M1 only" : \
                            (DEMO_ONLY_MOTOR == 2) ? "M2 only" : "M1 + M2")

typedef enum { DEMO_IDLE, DEMO_SELFTEST, DEMO_RUNNING, DEMO_FAULT, DEMO_PINTEST } demo_state_t;

#define PINTEST_STEP_MS    5000U

static const char *const k_pin_names[] = {
  "all four IN pins LOW",
  "M1IN1 HIGH (Nucleo D7 / PA8)",
  "M1IN2 HIGH (Nucleo D8 / PA9)",
  "M2IN1 HIGH (Nucleo D2 / PA10)",
  "M2IN2 HIGH (CN10 pin 14 / PA11)",
};
#define PINTEST_STEPS (sizeof(k_pin_names) / sizeof(k_pin_names[0]))

typedef struct {
  const char *name;
  int8_t      m1;   /* +1 forward, -1 reverse (M1 = left wheel) */
  int8_t      m2;   /* M2 = right wheel */
} phase_t;

#if DEMO_SPIN
/* Played forwards, then backwards with every direction flipped, so the
 * robot ends where it started: spin L, spin R, undo R, undo L. */
static const phase_t k_phases[] = {
  { "spin left",                  -1, +1 },
  { "spin right",                 +1, -1 },
};
#define PHASE_COUNT (2U * (sizeof(k_phases) / sizeof(k_phases[0])))
#else
static const phase_t k_phases[] = {
  { "both forward",               +1, +1 },
  { "both reverse",               -1, -1 },
};
#define PHASE_COUNT (DEMO_ONE_SHOT ? 1U : (sizeof(k_phases) / sizeof(k_phases[0])))
#endif
#define TABLE_COUNT (sizeof(k_phases) / sizeof(k_phases[0]))

static demo_state_t s_state;
static uint32_t s_last_tick;
static uint32_t s_last_ms;
static uint32_t s_last_telem;
static uint32_t s_run_start;
static uint32_t s_st_time;                  /* start of the current self-test stage */
static uint8_t  s_st_step;
static bool     s_fault_logged[MOTOR_COUNT];
static uint32_t s_phase;
static bool     s_btn_prev_pressed;
static uint8_t  s_btn_stable_ticks;
static bool     s_btn_latched;

/* ---------------------------------------------------------------------- */

/** Move number `i` of the sequence (0 .. PHASE_COUNT-1). In the spin demo
 *  the second half replays the table backwards with directions flipped. */
static void phase_get(uint32_t i, int8_t *m1, int8_t *m2, const char **name, bool *undo)
{
  *undo = false;
  if (i >= TABLE_COUNT)
  {
    i = (2U * TABLE_COUNT) - 1U - i;      /* only reached in the spin demo */
    *undo = true;
  }
  *m1   = *undo ? (int8_t)-k_phases[i].m1 : k_phases[i].m1;
  *m2   = *undo ? (int8_t)-k_phases[i].m2 : k_phases[i].m2;
  *name = k_phases[i].name;
}

/** Magnitude envelope (permille) at time t within one phase. */
static int16_t envelope(uint32_t t)
{
  if (t < RAMP_MS)
    return (int16_t)(((int32_t)DEMO_MAX_DUTY * (int32_t)t) / (int32_t)RAMP_MS);
  t -= RAMP_MS;
  if (t < HOLD_MS)
    return DEMO_MAX_DUTY;
  t -= HOLD_MS;
  if (t < RAMP_MS)
    return (int16_t)(((int32_t)DEMO_MAX_DUTY * (int32_t)(RAMP_MS - t)) / (int32_t)RAMP_MS);
  return 0;
}

/** Debounced press detection on B1 (active low). Returns true once per press. */
static bool button_pressed_edge(void)
{
  bool raw = (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN) == GPIO_PIN_RESET);

  if (raw == s_btn_prev_pressed)
  {
    if (s_btn_stable_ticks < 255U) s_btn_stable_ticks++;
  }
  else
  {
    s_btn_stable_ticks = 0U;
    s_btn_prev_pressed = raw;
  }

  /* 3 stable ticks = 30 ms */
  if (s_btn_stable_ticks == 3U)
  {
    if (raw && !s_btn_latched) { s_btn_latched = true;  return true; }
    if (!raw)                  { s_btn_latched = false; }
  }
  return false;
}

/** Prints e.g. "+183.4" from a value in tenths, without float printf. */
static void print_fixed1(int32_t value_x10)
{
  char sign = (value_x10 < 0) ? '-' : '+';
  int32_t a = labs(value_x10);
  uart_log("%c%ld.%ld", sign, (long)(a / 10), (long)(a % 10));
}

static int32_t rpm_x10(int32_t delta, uint32_t dt_ms)
{
  if (dt_ms == 0U) return 0;
  /* output-shaft rpm x10 = delta / 1920 rev per dt ms * 60000 ms/min * 10 */
  return (int32_t)(((int64_t)delta * 600000LL) /
                   ((int64_t)ENC_COUNTS_PER_OUTPUT_REV * (int64_t)dt_ms));
}

static void print_motor(const char *label, uint8_t m, int32_t delta, uint32_t dt)
{
  uart_log("  %s ", label);
  print_fixed1(motor_get_duty(m));     /* permille -> percent x10 */
  uart_log("%% ");
  print_fixed1(rpm_x10(delta, dt));
  uart_log("rpm");
}

static void print_time(uint32_t now)
{
  uart_log("t=%5lu.%02lus", (unsigned long)(now / 1000U), (unsigned long)((now % 1000U) / 10U));
}

static void enter_idle(void)
{
  motor_enable(false);
  s_state = DEMO_IDLE;
  uart_log("\n[idle] motors off. Press B1 to start. Turn a wheel by hand to test its encoder.\n");
}

/** Stage 1 of the start-up self-test: awake (EN HIGH) but disabled (D2 LOW). */
static void enter_selftest(uint32_t now)
{
  motor_wake();
  s_st_time = now;
  s_st_step = 0U;
  s_state   = DEMO_SELFTEST;
  uart_log("\n[self-test] before any PWM (0%% duty)\n");
}

/** Driver is already enabled (D2 HIGH) by the self-test; start the profile. */
static void enter_running(uint32_t now)
{
  s_run_start = now;
  s_phase     = PHASE_COUNT;   /* forces the first phase banner */
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) s_fault_logged[m] = false;
  s_state = DEMO_RUNNING;
  uart_log("\n[run] %s: %u move(s), %lu ms at %d%% duty each%s. B1 stops.\n",
           DEMO_SPIN ? "spin demo" : "straight demo", (unsigned)PHASE_COUNT,
           (unsigned long)HOLD_MS, DEMO_MAX_DUTY / 10,
           DEMO_ONE_SHOT ? ", then idle" : ", repeating");
}

/** Shows the current IN-pin test step; step 0 = all LOW, 1..4 = one pin HIGH. */
static void pintest_show(uint8_t step)
{
  motor_test_in_pin((uint8_t)(step - 1U));   /* step 0 -> 255 = all LOW */
  uart_log("[pin test %u/%u] %s; the other IN pins LOW\n",
           (unsigned)(step + 1U), (unsigned)PINTEST_STEPS, k_pin_names[step]);
}

static void enter_pintest(uint32_t now)
{
  motor_enable(false);          /* EN LOW: driver asleep, motors cannot move */
  s_st_time = now;
  s_st_step = 0U;
  s_state   = DEMO_PINTEST;
  uart_log("\n[pin test] driver asleep. Measure DC volts at each driver IN pin\n"
           "           (black probe on GND). Exactly the pin named should read ~3.3 V,\n"
           "           the rest 0 V. %lu s per step, repeats. Press B1 to stop.\n",
           (unsigned long)(PINTEST_STEP_MS / 1000U));
  pintest_show(0U);
}

static const char *sf_level(bool low) { return low ? "LOW " : "HIGH"; }

/** Runs one self-test stage once its settle time has passed. */
static void selftest_step(uint32_t now)
{
  bool raw1 = motor_sf_pin_low(MOTOR_1);
  bool raw2 = motor_sf_pin_low(MOTOR_2);
  uint8_t c = motor_ctrl_readback();

  if (s_st_step == 0U)
  {
    /* EN HIGH, D2 LOW = "disabled": the truth table says SF must be LOW. */
    uart_log("  1/2 EN=%u D1=%u D2=%u at MCU   M1SF=%s M2SF=%s  (expect LOW  LOW )\n",
             (c & MOTOR_CTRL_EN) ? 1U : 0U, (c & MOTOR_CTRL_D1) ? 1U : 0U,
             (c & MOTOR_CTRL_D2) ? 1U : 0U, sf_level(raw1), sf_level(raw2));
    /* Only the channels under test count; "l" here means "wrongly HIGH". */
    bool l1 = raw1 || !USE_M1;
    bool l2 = raw2 || !USE_M2;
    if (!l1 || !l2)
    {
      uart_log("      WARNING: %s%s%s not LOW with D2 LOW. That SF wire is open or on\n",
               !l1 ? "M1SF" : "", (!l1 && !l2) ? " and " : "", !l2 ? "M2SF" : "");
      uart_log("      the wrong pin, EN is not reaching the driver, or there is no VIN.\n");
      uart_log("      Fault detection on that channel cannot be trusted.\n");
    }
    motor_enable(true);          /* D2 HIGH, still 0 % duty */
    s_st_time = now;
    s_st_step = 1U;
    return;
  }

  /* EN HIGH, D1 LOW, D2 HIGH, both INs LOW (brake): SF must be HIGH. */
  uart_log("  2/2 EN=%u D1=%u D2=%u at MCU   M1SF=%s M2SF=%s  (expect HIGH HIGH)\n",
           (c & MOTOR_CTRL_EN) ? 1U : 0U, (c & MOTOR_CTRL_D1) ? 1U : 0U,
           (c & MOTOR_CTRL_D2) ? 1U : 0U, sf_level(raw1), sf_level(raw2));
  bool l1 = raw1 && USE_M1;      /* ignore the channel not under test */
  bool l2 = raw2 && USE_M2;
  if (l1 || l2)
  {
    uart_log("      FAIL: %s%s%s still LOW with D2 HIGH and no PWM yet.\n",
             l1 ? "M1SF" : "", (l1 && l2) ? " and " : "", l2 ? "M2SF" : "");
    uart_log("      Not caused by PWM or motor current. Check that channel's D2 (HIGH)\n");
    uart_log("      and D1 (LOW) AT THE DRIVER PIN, VIN > 5 V, and for an OUT pin\n");
    uart_log("      shorted to VIN (both low-side FETs are on at 0%% duty).\n");
    if (DEMO_STOP_ON_FAULT)
    {
      s_state = DEMO_FAULT;
      uart_log("      Not starting. Press B1 to return to idle.\n");
      return;
    }
    uart_log("      Fault stop is OFF: running anyway.\n");
  }
  else
  {
    uart_log("      pass\n");
  }
  enter_running(now);
}

/** Prints, once per run and per channel, the conditions at the moment a
 *  fault latched (SF LOW on MOTOR_FAULT_MS consecutive 1 ms samples). */
static void log_new_faults(uint32_t now)
{
  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    if ((m == MOTOR_1 && !USE_M1) || (m == MOTOR_2 && !USE_M2)) continue;
    if (!motor_fault(m) || s_fault_logged[m]) continue;
    s_fault_logged[m] = true;
    uint32_t t = now - s_run_start;
    uart_log("\n[sf] M%uSF LOW >= %u ms at run +%lu.%03lu s, phase %lu, duty",
             (unsigned)(m + 1U), (unsigned)MOTOR_FAULT_MS,
             (unsigned long)(t / 1000U), (unsigned long)(t % 1000U),
             (unsigned long)(((t / PHASE_MS) % PHASE_COUNT) + 1U));
    uart_log(" M1 ");
    print_fixed1(motor_get_duty(MOTOR_1));
    uart_log("%% M2 ");
    print_fixed1(motor_get_duty(MOTOR_2));
    uart_log("%%; other SF %s\n",
             motor_sf_pin_low((uint8_t)(1U - m)) ? "LOW too" : "ok");
  }
}

static void enter_fault(void)
{
  bool f1 = USE_M1 && motor_fault(MOTOR_1);
  bool f2 = USE_M2 && motor_fault(MOTOR_2);
  motor_set_duty(MOTOR_1, 0);
  motor_set_duty(MOTOR_2, 0);
  s_state = DEMO_FAULT;
  uart_log("\n[FAULT] MC33926 reported a fault on %s%s%s:\n"
           "        short/over-current, over-temperature or VIN under-voltage.\n"
           "        Both motors stopped. Check battery + wiring, then press B1 to clear.\n",
           f1 ? "M1" : "", (f1 && f2) ? " and " : "", f2 ? "M2" : "");
}

/* ---------------------------------------------------------------------- */

void motor_demo_init(void)
{
  uart_log_init();
  motor_init();
  encoder_init();

  s_btn_prev_pressed = false;
  s_btn_stable_ticks = 0U;
  s_btn_latched      = false;
  s_last_tick  = HAL_GetTick();
  s_last_ms    = s_last_tick;
  s_last_telem = s_last_tick;

  uart_log("\n\n=== SnackBot P&M two-motor demo (Nucleo-F446RE) ===\n");
  uart_log("SYSCLK %lu Hz, PWM %u Hz on IN pins (TIM1 CH1-4), D2 held HIGH\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned)MOTOR_PWM_FREQ_HZ);
  uart_log("Encoders: TIM3 (M1, D12/D11) + M2 on " ENC2_PINS_NAME ", %d counts per output rev. WHEELS OFF THE GROUND.\n",
           ENC_COUNTS_PER_OUTPUT_REV);
  uart_log("Fault stop: %s\n", DEMO_STOP_ON_FAULT ? "ON (SF on D5 / A3)" : "OFF");
  uart_log("Motors under test: %s\n", MOTORS_LABEL);
  if (DEMO_PIN_TEST) uart_log("B1 runs the IN-pin WIRING TEST (DEMO_PIN_TEST 1), not the demo.\n");
  enter_idle();
}

void motor_demo_poll(void)
{
  uint32_t now = HAL_GetTick();

  /* ---- every 1 ms: debounce the SF fault lines ------------------------
   * One fresh sample per pass, never a burst: after a blocking log line
   * (several ms) the missed ticks are NOT replayed against a single
   * reading, so a fault always needs MOTOR_FAULT_MS separate samples that
   * are at least 1 ms apart. No settle delay is needed: the self-test has
   * already seen SF HIGH with the driver enabled. */
  if (s_last_ms != now)
  {
    s_last_ms = now;
    if (s_state == DEMO_RUNNING)
    {
      motor_fault_poll();
      log_new_faults(now);
    }
  }

  /* ---- every 10 ms: button, profile, fault check, LED ----------------- */
  if ((now - s_last_tick) >= TICK_MS)
  {
    s_last_tick = now;
    bool pressed = button_pressed_edge();

    switch (s_state)
    {
      case DEMO_IDLE:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ((now / 500U) % 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (pressed)
        {
          if (DEMO_PIN_TEST) enter_pintest(now);
          else               enter_selftest(now);
        }
        break;

      case DEMO_SELFTEST:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        if (pressed)
        {
          enter_idle();
          break;
        }
        if ((now - s_st_time) >= SELFTEST_STEP_MS) selftest_step(now);
        break;

      case DEMO_RUNNING:
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        if (pressed)
        {
          enter_idle();
          break;
        }
        if (DEMO_STOP_ON_FAULT && ((USE_M1 && motor_fault(MOTOR_1)) ||
                                   (USE_M2 && motor_fault(MOTOR_2))))
        {
          enter_fault();
          break;
        }
        uint32_t t     = now - s_run_start;
        /* One-shot: stop once the last move has ramped down (no final pause). */
        if (DEMO_ONE_SHOT && t >= ((PHASE_COUNT - 1U) * PHASE_MS + MOVE_MS))
        {
          uart_log("\n[done] sequence complete.\n");
          enter_idle();          /* duty 0, driver asleep; B1 starts the next run */
          break;
        }
        uint32_t phase = (t / PHASE_MS) % PHASE_COUNT;
        int16_t  env   = envelope(t % PHASE_MS);
        int8_t   d1, d2;
        const char *name;
        bool     undo;
        phase_get(phase, &d1, &d2, &name, &undo);
        if (phase != s_phase)
        {
          s_phase = phase;
          if (DEMO_ONLY_MOTOR == 0)
          {
            uart_log("\n[move %lu/%u] %s%s\n", (unsigned long)(phase + 1U),
                     (unsigned)PHASE_COUNT, undo ? "undo " : "", name);
          }
          else
          {
            int8_t dir = USE_M1 ? d1 : d2;
            uart_log("\n[move %lu/%u] %s: %s\n", (unsigned long)(phase + 1U),
                     (unsigned)PHASE_COUNT, MOTORS_LABEL, (dir > 0) ? "forward" : "reverse");
          }
        }
        motor_set_duty(MOTOR_1, USE_M1 ? (int16_t)(env * d1) : 0);
        motor_set_duty(MOTOR_2, USE_M2 ? (int16_t)(env * d2) : 0);
        break;
      }

      case DEMO_PINTEST:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ((now / 250U) % 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (pressed)
        {
          motor_test_in_pin(0xFFU);
          enter_idle();
          break;
        }
        if ((now - s_st_time) >= PINTEST_STEP_MS)
        {
          s_st_time = now;
          s_st_step = (uint8_t)((s_st_step + 1U) % PINTEST_STEPS);
          pintest_show(s_st_step);
        }
        break;

      case DEMO_FAULT:
      default:
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ((now / 100U) % 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (pressed)
        {
          enter_idle();          /* EN low -> MC33926 sleep clears the latched fault */
        }
        break;
    }
  }

  /* ---- 100 ms: speed estimate + telemetry ---------------------------- */
  uint32_t dt = now - s_last_telem;
  if (dt >= TELEMETRY_MS)
  {
    s_last_telem = now;
    int32_t d1 = encoder_update(0);
    int32_t d2 = encoder_update(1);
    if (s_state == DEMO_RUNNING)
    {
      print_time(now);
      print_motor("M1", MOTOR_1, d1, dt);
      print_motor("  M2", MOTOR_2, d2, dt);
      bool l1 = motor_sf_pin_low(MOTOR_1);
      bool l2 = motor_sf_pin_low(MOTOR_2);
      uart_log("  sf=%s\n", (l1 && l2) ? "M1+M2 LOW" : l1 ? "M1 LOW" : l2 ? "M2 LOW" : "ok");
    }
    else if (d1 != 0 || d2 != 0)
    {
      print_time(now);
      uart_log("  idle  M1 pos=%ld  M2 pos=%ld\n",
               (long)encoder_position(0), (long)encoder_position(1));
    }
  }
}
