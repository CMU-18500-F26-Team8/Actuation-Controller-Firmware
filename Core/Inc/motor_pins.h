/**
 * @file    motor_pins.h
 * @brief   Pin map for the two-motor setup (Nucleo-F446RE + Pololu #1213
 *          Dual MC33926 carrier + 2x Pololu #4752 gearmotors).
 *
 * Drive scheme: PWM on IN1/IN2 (sign-magnitude, brake during off-time),
 * D2 held HIGH and D1 held LOW while running. All four PWM signals come
 * from TIM1 so both bridges switch in step.
 *
 *   Nucleo pin      MCU    Function             Driver / encoder
 *   D7              PA8    TIM1_CH1 (AF1)       M1IN1
 *   D8              PA9    TIM1_CH2 (AF1)       M1IN2
 *   D2              PA10   TIM1_CH3 (AF1)       M2IN1
 *   CN10 pin 14     PA11   TIM1_CH4 (AF1)       M2IN2
 *   D4              PB5    GPIO out             M1D2 + M2D2 (HIGH = enabled)
 *   D6              PB10   GPIO out             M1D1 + M2D1 (held LOW)
 *   D9              PC7    GPIO out             EN (both channels)
 *   D5              PB4    GPIO in, pull-up     M1SF (LOW = fault)
 *   A3              PB0    GPIO in, pull-up     M2SF (LOW = fault)
 *   D12 / D11       PA6/7  TIM3_CH1/2 (AF2)     encoder 1 A / B
 *   D10 / CN7-21    PB6/7  TIM4_CH1/2 (AF2)     encoder 2 A / B   (ENC2_ALT_PINS 0)
 *   CN7-17 / D3     PA15/PB3 TIM2_CH1/2 (AF1)   encoder 2 A / B   (ENC2_ALT_PINS 1)
 *   A0              PA0    UART4_TX (AF8)       Pi GPIO15 RXD (pin 10)
 *   A1              PA1    UART4_RX (AF8)       Pi GPIO14 TXD (pin 8)
 *   (UART4 is configured in pi_link.c)
 *
 * All of these are 5 V-tolerant (FT) per the STM32F446 datasheet. The
 * encoders run from 5 V, so their inputs have no internal pull resistors.
 *
 * Pins are configured in code (motor.c / encoder.c), not in CubeMX, so
 * regenerating from f446_blinky.ioc does not touch them.
 */
#ifndef MOTOR_PINS_H
#define MOTOR_PINS_H

#include "stm32f4xx_hal.h"

/* ---- PWM: TIM1 channels 1-4 on PA8..PA11 ------------------------------ */
#define MOT_PWM_PORT       GPIOA
#define MOT_PWM_PINS       (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
#define MOT_PWM_AF         GPIO_AF1_TIM1

/* ---- Shared driver control ------------------------------------------- */
#define MD_D2_PORT         GPIOB
#define MD_D2_PIN          GPIO_PIN_5      /* D4  -> M1D2 + M2D2 */
#define MD_D1_PORT         GPIOB
#define MD_D1_PIN          GPIO_PIN_10     /* D6  -> M1D1 + M2D1 */
#define MD_EN_PORT         GPIOC
#define MD_EN_PIN          GPIO_PIN_7      /* D9  -> EN */

/* ---- Status flags ------------------------------------------------------ */
#define M1_SF_PORT         GPIOB
#define M1_SF_PIN          GPIO_PIN_4      /* D5  <- M1SF */
#define M2_SF_PORT         GPIOB
#define M2_SF_PIN          GPIO_PIN_0      /* A3  <- M2SF */

/* ---- Encoders ------------------------------------------------------------ */
#define ENC1_PORT          GPIOA
#define ENC1_PINS          (GPIO_PIN_6 | GPIO_PIN_7)   /* D12, D11 */
#define ENC1_AF            GPIO_AF2_TIM3
/* Encoder 2 pins. 0 = TIM4 on D10 (A, yellow) + CN7 pin 21 (B, white).
 * 1 = TIM2 on CN7 pin 17 (PA15, A, yellow) + D3 (PB3, B, white), for when
 * D10 is unusable. Both pins move together: a timer's encoder mode needs
 * its own CH1 and CH2. PA15/PB3 are JTAG pins; SWD debugging is unaffected. */
#define ENC2_ALT_PINS      1

#if ENC2_ALT_PINS
#define ENC2_TIM           TIM2
#define ENC2_A_PORT        GPIOA
#define ENC2_A_PIN         GPIO_PIN_15                 /* CN7 pin 17 */
#define ENC2_B_PORT        GPIOB
#define ENC2_B_PIN         GPIO_PIN_3                  /* D3 */
#define ENC2_AF            GPIO_AF1_TIM2
#define ENC2_TIM_CLK_ENABLE()   __HAL_RCC_TIM2_CLK_ENABLE()
#define ENC2_TIM_RESET()        do { __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET(); } while (0)
#define ENC2_PINS_NAME     "CN7-17/D3"
#else
#define ENC2_TIM           TIM4
#define ENC2_A_PORT        GPIOB
#define ENC2_A_PIN         GPIO_PIN_6                  /* D10 */
#define ENC2_B_PORT        GPIOB
#define ENC2_B_PIN         GPIO_PIN_7                  /* CN7 pin 21 */
#define ENC2_AF            GPIO_AF2_TIM4
#define ENC2_TIM_CLK_ENABLE()   __HAL_RCC_TIM4_CLK_ENABLE()
#define ENC2_TIM_RESET()        do { __HAL_RCC_TIM4_FORCE_RESET(); __HAL_RCC_TIM4_RELEASE_RESET(); } while (0)
#define ENC2_PINS_NAME     "D10/CN7-21"
#endif

#endif /* MOTOR_PINS_H */
