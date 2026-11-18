/**
 * @file    motor.h
 * @brief   Two-channel driver for the Pololu Dual MC33926 carrier.
 *
 * Drive scheme (sign-magnitude, "drive/brake"):
 *   forward : IN1 = PWM, IN2 = LOW      reverse : IN1 = LOW, IN2 = PWM
 *   PWM off-time: both INs LOW = low-side brake (slow decay), which gives a
 *   speed that is close to proportional to duty.
 *   D2 = HIGH and D1 = LOW while enabled, so the outputs are never in a
 *   "disabled" state while running and SF only goes LOW on a real fault
 *   (short/over-current, over-temperature, VIN under-voltage).
 *   EN LOW = both driver chips asleep; outputs high-Z (wheels coast).
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_COUNT         2
#define MOTOR_1             0
#define MOTOR_2             1

#define MOTOR_PWM_FREQ_HZ   20000U   /* MC33926 max; SLEW pin must be HIGH */
#define MOTOR_DUTY_MAX      1000     /* duty is in permille: -1000 .. +1000 */

/* Flip a motor's sense of "positive" without rewiring. When the motors are
 * mounted mirror-image on the chassis, set one of these to 1 AND flip the
 * matching ENCx_DIRECTION in encoder.h so its speed sign follows. */
#define MOTOR1_INVERT       0
#define MOTOR2_INVERT       1

/** GPIOs + TIM1 4-channel PWM. Leaves the driver asleep with 0 % duty. */
void motor_init(void);

/** true: D1 LOW, D2 HIGH, EN HIGH (awake, braking at 0 %).
 *  false: duty 0, D2 LOW, EN LOW (asleep, wheels coast). Sleeping also
 *  clears latched MC33926 faults and the fault flags below. */
void motor_enable(bool on);
bool motor_is_enabled(void);

/** Self-test stage: EN HIGH, D1 LOW, D2 still LOW (awake but disabled).
 *  Per the MC33926 truth table both SF lines must read LOW in this state,
 *  which proves each SF wire is on an SF pin and the chip has VIN.
 *  Follow with motor_enable(true) to raise D2. */
void motor_wake(void);

/** Levels actually present on the MCU's control pins (read from IDR, so a
 *  shorted or stuck pin shows up). Diagnostics only. */
#define MOTOR_CTRL_EN       0x01U
#define MOTOR_CTRL_D1       0x02U
#define MOTOR_CTRL_D2       0x04U
uint8_t motor_ctrl_readback(void);

/** Signed duty in permille for one motor (MOTOR_1 / MOTOR_2).
 *  Positive = IN1 PWM (before MOTORx_INVERT). Clamped to +/-1000. */
void motor_set_duty(uint8_t motor, int16_t duty_permille);
int16_t motor_get_duty(uint8_t motor);

/** Wiring test: drive one IN pin steadily HIGH and the other three LOW
 *  (0 = M1IN1/PA8, 1 = M1IN2/PA9, 2 = M2IN1/PA10, 3 = M2IN2/PA11, any
 *  other value = all LOW). Call only with the driver asleep (EN LOW). */
void motor_test_in_pin(uint8_t pin);

/** Call at most once per millisecond while enabled (one fresh sample per
 *  call). Latches a fault after SF has been LOW on MOTOR_FAULT_MS
 *  consecutive calls. */
#define MOTOR_FAULT_MS      3U
void motor_fault_poll(void);

/** Latched fault for one motor (cleared by motor_enable()). */
bool motor_fault(uint8_t motor);

/** Raw SF pin level, LOW = true. Diagnostics only. */
bool motor_sf_pin_low(uint8_t motor);

#endif /* MOTOR_H */
