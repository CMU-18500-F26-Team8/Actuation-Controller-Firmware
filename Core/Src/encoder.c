/**
 * @file    encoder.c
 * @brief   TIM3 and ENC2_TIM (TIM4, or TIM2 with ENC2_ALT_PINS) in encoder
 *          mode 3 (count both edges of TI1 and TI2).
 */
#include "encoder.h"
#include "motor_pins.h"

static TIM_TypeDef *const k_tim[ENCODER_COUNT] = { TIM3, ENC2_TIM };
static const int8_t k_dir[ENCODER_COUNT] = { ENC1_DIRECTION, ENC2_DIRECTION };

static uint16_t s_last_cnt[ENCODER_COUNT];
static int32_t  s_position[ENCODER_COUNT];

static void encoder_timer_init(TIM_TypeDef *t)
{
  t->CR1 = 0U;
  t->PSC = 0U;
  t->ARR = 0xFFFFU;
  /* CC1S = 01 (IC1 <- TI1), CC2S = 01 (IC2 <- TI2).
   * Input filter 0b1000: fSAMPLING = fDTS/8, N = 6 (~0.6 us at 84 MHz),
   * rejects short glitches; real edges are >= ~95 us apart at 330 rpm. */
  t->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
             (0x8U << TIM_CCMR1_IC1F_Pos) | (0x8U << TIM_CCMR1_IC2F_Pos);
  t->CCER  = 0U;                                   /* non-inverted */
  t->SMCR  = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;      /* encoder mode 3 */
  t->CNT   = 0U;
  t->CR1   = TIM_CR1_CEN;
}

void encoder_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* No internal pull resistors: the encoders run from 5 V and these FT
   * pins must not have pulls enabled when driven above VDD. */
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = ENC1_AF;
  g.Pin = ENC1_PINS;
  HAL_GPIO_Init(ENC1_PORT, &g);
  g.Alternate = ENC2_AF;
  g.Pin = ENC2_A_PIN;
  HAL_GPIO_Init(ENC2_A_PORT, &g);
  g.Pin = ENC2_B_PIN;
  HAL_GPIO_Init(ENC2_B_PORT, &g);

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();
  ENC2_TIM_CLK_ENABLE();
  ENC2_TIM_RESET();

  for (uint8_t i = 0; i < ENCODER_COUNT; i++)
  {
    encoder_timer_init(k_tim[i]);
    s_last_cnt[i] = 0U;
    s_position[i] = 0;
  }
}

int32_t encoder_update(uint8_t idx)
{
  if (idx >= ENCODER_COUNT) return 0;
  uint16_t cnt = (uint16_t)k_tim[idx]->CNT;
  /* Unsigned subtraction then cast to int16 handles wrap-around. */
  int32_t delta = (int16_t)(uint16_t)(cnt - s_last_cnt[idx]);
  s_last_cnt[idx] = cnt;
  delta *= k_dir[idx];
  s_position[idx] += delta;
  return delta;
}

int32_t encoder_position(uint8_t idx)
{
  return (idx < ENCODER_COUNT) ? s_position[idx] : 0;
}
