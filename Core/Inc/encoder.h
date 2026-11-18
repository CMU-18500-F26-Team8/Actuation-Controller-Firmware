/**
 * @file    encoder.h
 * @brief   Two quadrature encoders: encoder 1 on TIM3, encoder 2 on TIM4
 *          (hardware x4 decoding).
 *
 * Pololu #4752: 64 counts/rev at the motor shaft (both edges, both
 * channels) x 30:1 gearbox = 1920 counts per output-shaft revolution.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_COUNT              2
#define ENC_COUNTS_PER_MOTOR_REV   64
#define GEAR_RATIO                 30
#define ENC_COUNTS_PER_OUTPUT_REV  (ENC_COUNTS_PER_MOTOR_REV * GEAR_RATIO)  /* 1920 */

/* Set to -1 if a positive duty on that motor makes its count go DOWN
 * (or swap that encoder's yellow/white wires). If you set MOTORx_INVERT
 * in motor.h for a mirrored motor, flip the matching value here too. */
#define ENC1_DIRECTION             (+1)
#define ENC2_DIRECTION             (-1)   /* checked on the robot with encoder 2
                                             * on CN7-17 / D3 (ENC2_ALT_PINS 1) */

void    encoder_init(void);

/** Read encoder `idx` (0 or 1), accumulate into its 32-bit position and
 *  return the change since the last call. Must be called more often than
 *  every ~3 s (32767 counts at full speed). */
int32_t encoder_update(uint8_t idx);

/** Accumulated position in counts (1920 counts = 1 output-shaft rev). */
int32_t encoder_position(uint8_t idx);

#endif /* ENCODER_H */
