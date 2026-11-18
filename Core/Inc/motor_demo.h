/**
 * @file    motor_demo.h
 * @brief   Bare-metal two-motor bring-up demo.
 *
 *   B1 (blue button)  : start / stop the demo; clears a latched fault
 *   LD2 (green LED)   : slow blink = idle, solid = running, fast blink = fault
 *   USART2 115200 8N1 : telemetry every 100 ms while running; while idle,
 *                       a line is printed whenever either encoder moves
 *                       (turn a wheel by hand to test its encoder).
 *
 * Each B1 press: self-test, then a sequence of moves, then back to idle.
 * Every move = ramp up 0.25 s, hold at DEMO_MAX_DUTY for DEMO_HOLD_MS
 * (DEMO_SPIN_HOLD_MS in the spin demo),
 * ramp down 0.25 s, pause 0.5 s.
 *   DEMO_SPIN 0: both motors forward (one move).
 *   DEMO_SPIN 1: spin left, spin right, then back through both spins in
 *                reverse order (undo right, undo left).
 * M1 = left wheel, M2 = right wheel. "Forward" = positive duty; M2 is
 * inverted for mirror mounting (MOTOR2_INVERT in motor.h).
 */
#ifndef MOTOR_DEMO_H
#define MOTOR_DEMO_H

/* Hold duty, permille (300 = 30 %, ~100 rpm no-load at 12 V). */
#define DEMO_MAX_DUTY       300

/* How long each move holds its duty, ms (not counting the 0.25 s ramps). */
#define DEMO_HOLD_MS        1000U

/* Spin demo only: how long each spin holds its duty, ms (used instead of
 * DEMO_HOLD_MS when DEMO_SPIN is 1). */
#define DEMO_SPIN_HOLD_MS   5000U

/* 0 = straight demo: both motors forward.
 * 1 = spin demo: spin left in place, spin right in place, then reverse back
 *     through both spins (undo the right spin, then undo the left spin). */
#define DEMO_SPIN           1

/* 1 = each B1 press runs the sequence once, then stops and waits for B1.
 * 0 = repeat until B1 is pressed again (the straight demo then alternates
 *     forward and reverse). */
#define DEMO_ONE_SHOT       1

/* 1 = stop both motors when either driver channel reports a fault on SF.
 * Works with PWM on IN1/IN2 because D2 is held HIGH while running. */
#define DEMO_STOP_ON_FAULT  1

/* Which motors the demo drives and checks:
 *   0 = both, 1 = motor 1 only, 2 = motor 2 only.
 * The other channel is held at 0 % duty and its SF line is printed but
 * ignored by the self-test verdict and the fault stop. */
#define DEMO_ONLY_MOTOR     0

/* 1 = B1 starts an IN-pin wiring test instead of the demo. The driver stays
 * asleep (EN LOW, motors never move) and the four IN pins are driven HIGH
 * one at a time, 5 s each, steady DC: measure each driver IN pin with a
 * multimeter and check exactly one reads ~3.3 V, the one named in the log. */
#define DEMO_PIN_TEST       0

void motor_demo_init(void);

/** Call continuously from the main while(1) loop. Non-blocking. */
void motor_demo_poll(void);

#endif /* MOTOR_DEMO_H */
