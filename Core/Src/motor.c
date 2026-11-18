/**
 * @file    motor.c
 * @brief   Two-channel MC33926 driver: TIM1 CH1-CH4 PWM on the IN pins.
 *
 * TIM1 is configured directly through its registers so this file does not
 * depend on the HAL TIM module being enabled in CubeMX.
 *
 * Why PWM on IN1/IN2 and not on D2: the MC33926 pulls SF LOW whenever its
 * outputs are disabled (D2 LOW or D1 HIGH), so PWM on D2 made SF useless as
 * a fault flag (bench result, 10 Nov 2026: SF never read HIGH while D2 was
 * switching). With D2 held HIGH, SF only goes LOW on a real fault, so a
 * plain debounced read works.
 */
#include "motor.h"
#include "motor_pins.h"

static int16_t  s_duty[MOTOR_COUNT];        /* logical duty, before invert */
static uint32_t s_arr;
static bool     s_enabled;
static bool     s_fault[MOTOR_COUNT];
static uint8_t  s_sf_low_ms[MOTOR_COUNT];

static const int8_t k_invert[MOTOR_COUNT] = {
  MOTOR1_INVERT ? -1 : 1,
  MOTOR2_INVERT ? -1 : 1,
};

/* TIM1 sits on APB2. Timer clock = PCLK2 if the APB2 prescaler is 1,
 * otherwise 2 x PCLK2 (RM0390 "Clocks" section). */
static uint32_t tim1_clock_hz(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
  return (ppre2 < 4U) ? pclk2 : (2U * pclk2);   /* 0xx = not divided */
}

void motor_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Safe levels before the pins become outputs: asleep, disabled. */
  HAL_GPIO_WritePin(MD_EN_PORT, MD_EN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MD_D2_PORT, MD_D2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MD_D1_PORT, MD_D1_PIN, GPIO_PIN_RESET);

  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = MD_EN_PIN;  HAL_GPIO_Init(MD_EN_PORT, &g);
  g.Pin = MD_D2_PIN;  HAL_GPIO_Init(MD_D2_PORT, &g);
  g.Pin = MD_D1_PIN;  HAL_GPIO_Init(MD_D1_PORT, &g);

  /* SF lines: open-drain on the carrier with a weak pull-up to VDD (3.3 V);
   * add the MCU pull-up too. An unplugged SF wire reads as "no fault". */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  g.Pin = M1_SF_PIN;  HAL_GPIO_Init(M1_SF_PORT, &g);
  g.Pin = M2_SF_PIN;  HAL_GPIO_Init(M2_SF_PORT, &g);

  /* PWM pins -> TIM1. Pull-downs keep IN1/IN2 LOW if the timer isn't
   * driving them (D2 is LOW then anyway, so the outputs are off). */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLDOWN;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = MOT_PWM_AF;
  g.Pin       = MOT_PWM_PINS;
  HAL_GPIO_Init(MOT_PWM_PORT, &g);

  /* ---- TIM1: edge-aligned up-counting PWM, channels 1-4 ---------------- */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();

  s_arr = (tim1_clock_hz() / MOTOR_PWM_FREQ_HZ) - 1U;  /* 84 MHz -> 4199 */

  TIM1->PSC  = 0U;
  TIM1->ARR  = s_arr;
  TIM1->RCR  = 0U;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  /* All four channels: PWM mode 1 (OCxM = 110), preload enabled. */
  TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE |
                TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
  /* OISx = 0: when outputs are disabled, every IN idles LOW (= brake). */
  TIM1->CR2  = 0U;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  /* Advanced timer: main output enable required. OSSI = 1 drives the idle
   * level when MOE drops instead of releasing the pins. */
  TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSI;
  TIM1->DIER = 0U;
  TIM1->CR1  = TIM_CR1_ARPE;
  TIM1->EGR  = TIM_EGR_UG;                    /* load PSC/ARR/CCRx now */
  TIM1->SR   = 0U;

  /* Debugger safety: at a breakpoint TIM1 freezes and, being an advanced
   * timer, its outputs are forced to the idle level (all INs LOW), so both
   * motors brake instead of running at their last duty. */
  DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;

  TIM1->CR1 |= TIM_CR1_CEN;

  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    s_duty[m] = 0;
    s_fault[m] = false;
    s_sf_low_ms[m] = 0U;
  }
  s_enabled = false;
}

void motor_enable(bool on)
{
  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    s_fault[m] = false;
    s_sf_low_ms[m] = 0U;
  }

  if (on)
  {
    HAL_GPIO_WritePin(MD_D1_PORT, MD_D1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MD_EN_PORT, MD_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MD_D2_PORT, MD_D2_PIN, GPIO_PIN_SET);
    s_enabled = true;
  }
  else
  {
    motor_set_duty(MOTOR_1, 0);
    motor_set_duty(MOTOR_2, 0);
    HAL_GPIO_WritePin(MD_D2_PORT, MD_D2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MD_EN_PORT, MD_EN_PIN, GPIO_PIN_RESET);
    s_enabled = false;
  }
}

bool motor_is_enabled(void)
{
  return s_enabled;
}

void motor_wake(void)
{
  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    s_fault[m] = false;
    s_sf_low_ms[m] = 0U;
  }
  motor_set_duty(MOTOR_1, 0);
  motor_set_duty(MOTOR_2, 0);
  HAL_GPIO_WritePin(MD_D1_PORT, MD_D1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MD_D2_PORT, MD_D2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MD_EN_PORT, MD_EN_PIN, GPIO_PIN_SET);
  s_enabled = false;            /* outputs still disabled by D2 */
}

uint8_t motor_ctrl_readback(void)
{
  uint8_t r = 0U;
  if (HAL_GPIO_ReadPin(MD_EN_PORT, MD_EN_PIN) == GPIO_PIN_SET) r |= MOTOR_CTRL_EN;
  if (HAL_GPIO_ReadPin(MD_D1_PORT, MD_D1_PIN) == GPIO_PIN_SET) r |= MOTOR_CTRL_D1;
  if (HAL_GPIO_ReadPin(MD_D2_PORT, MD_D2_PIN) == GPIO_PIN_SET) r |= MOTOR_CTRL_D2;
  return r;
}

void motor_set_duty(uint8_t motor, int16_t duty)
{
  if (motor >= MOTOR_COUNT) return;

  if (duty > MOTOR_DUTY_MAX)  duty = MOTOR_DUTY_MAX;
  if (duty < -MOTOR_DUTY_MAX) duty = -MOTOR_DUTY_MAX;
  s_duty[motor] = duty;

  int32_t d = (int32_t)duty * k_invert[motor];
  uint32_t mag = (uint32_t)((d < 0) ? -d : d);
  uint32_t ccr = (mag * (s_arr + 1U)) / (uint32_t)MOTOR_DUTY_MAX;
  uint32_t in1 = (d > 0) ? ccr : 0U;
  uint32_t in2 = (d < 0) ? ccr : 0U;

  /* Block the update event while writing both compare registers so the
   * pair always changes in the same PWM period (never one old, one new). */
  TIM1->CR1 |= TIM_CR1_UDIS;
  if (motor == MOTOR_1)
  {
    TIM1->CCR1 = in1;
    TIM1->CCR2 = in2;
  }
  else
  {
    TIM1->CCR3 = in1;
    TIM1->CCR4 = in2;
  }
  TIM1->CR1 &= ~TIM_CR1_UDIS;
}

void motor_test_in_pin(uint8_t pin)
{
  /* CCR = ARR + 1 in PWM mode 1 keeps the output HIGH for the whole period. */
  uint32_t hi = s_arr + 1U;
  for (uint8_t m = 0; m < MOTOR_COUNT; m++) s_duty[m] = 0;
  TIM1->CR1 |= TIM_CR1_UDIS;
  TIM1->CCR1 = (pin == 0U) ? hi : 0U;
  TIM1->CCR2 = (pin == 1U) ? hi : 0U;
  TIM1->CCR3 = (pin == 2U) ? hi : 0U;
  TIM1->CCR4 = (pin == 3U) ? hi : 0U;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
}

int16_t motor_get_duty(uint8_t motor)
{
  return (motor < MOTOR_COUNT) ? s_duty[motor] : 0;
}

bool motor_sf_pin_low(uint8_t motor)
{
  if (motor == MOTOR_1)
    return HAL_GPIO_ReadPin(M1_SF_PORT, M1_SF_PIN) == GPIO_PIN_RESET;
  return HAL_GPIO_ReadPin(M2_SF_PORT, M2_SF_PIN) == GPIO_PIN_RESET;
}

void motor_fault_poll(void)
{
  if (!s_enabled) return;
  for (uint8_t m = 0; m < MOTOR_COUNT; m++)
  {
    if (motor_sf_pin_low(m))
    {
      if (s_sf_low_ms[m] < 255U) s_sf_low_ms[m]++;
      if (s_sf_low_ms[m] >= MOTOR_FAULT_MS) s_fault[m] = true;
    }
    else
    {
      s_sf_low_ms[m] = 0U;
    }
  }
}

bool motor_fault(uint8_t motor)
{
  return (motor < MOTOR_COUNT) ? s_fault[motor] : false;
}
