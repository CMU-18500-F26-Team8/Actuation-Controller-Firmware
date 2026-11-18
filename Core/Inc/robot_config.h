/**
 * @file    robot_config.h
 * @brief   Every tunable for Pi-controlled driving, in one place.
 *
 * Units are SI (m, m/s, s) unless the name says otherwise. Motor duty is in
 * permille: 1000 = 100 %. Change a value, rebuild, flash.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "motor.h"

/* ---- Mode ------------------------------------------------------------------
 * 0 = Pi control (UART4 commands, B1 = front button, LD2 = LEDs).
 * 1 = bench demo from motor_demo.c (B1 starts a run; see motor_demo.h). */
#define ROBOT_MODE              0

/* ---- Speeds ---------------------------------------------------------------- */
#define TARGET_SPEED_MPS        0.16f   /* straight and reverse, at each wheel */
#define TURN_SPEED_MPS          0.16f   /* wheel speed while spinning in place */
#define ACCEL_MPS2              0.4f    /* setpoint ramp limit (both wheels).
                                         * Large (e.g. 100) = a true step */

/* ---- Speed PID, one per wheel ---------------------------------------------
 * duty = KP*err + KI*integral(err) + KD*d(err)/dt      (each wheel on its own)
 * err = setpoint - measured speed, in m/s; duty in permille (1000 = 100 %).
 * The integral term supplies the steady duty the wheel needs (~120 permille
 * at 0.16 m/s unloaded), so KI must be non-zero to reach the target.
 * All gains, the speeds and ACCEL_MPS2 can be changed live over the USB
 * debug port (set,<name>,<value>) with tools/pid_tune.py; copy the values
 * you settle on back here. Current values: tuned on the robot, wheels in
 * the air. */
#define PID_L_KP                200.0f  /* left wheel:  permille per (m/s) */
#define PID_L_KI                1000.0f /*              permille per (m/s * s) */
#define PID_L_KD                0.0f    /*              permille per (m/s / s) */
#define PID_R_KP                200.0f  /* right wheel */
#define PID_R_KI                1000.0f
#define PID_R_KD                0.0f
#define DUTY_LIMIT              600     /* PID output clamp, permille (anti-windup) */

/* ---- Control loop timing --------------------------------------------------- */
#define CONTROL_PERIOD_MS       20U     /* 50 Hz */
#define SPEED_WINDOW_TICKS      5U      /* speed = counts over the last N ticks
                                         * (5 x 20 ms = 100 ms; at 0.16 m/s
                                         * one count is 4.5 % of a 20 ms window) */

/* ---- Geometry -------------------------------------------------------------- */
#define WHEEL_DIAMETER_M        0.0889f /* 3.5 in */
#define TRACK_WIDTH_M           0.2413f /* 9.5 in. If turns overshoot, raise it;
                                         * if they undershoot, lower it */
#define LEFT_MOTOR              MOTOR_2 /* also selects encoder 2 (checked on the robot) */
#define RIGHT_MOTOR             MOTOR_1

/* ---- Turns (always in place; positive angle = counter-clockwise) ---------- */
#define TURN_SLOW_DEG           15.0f   /* ramp the speed down over the last N deg */
#define TURN_MIN_SPEED_MPS      0.05f   /* speed at the very end of a turn */

/* ---- Pi link ---------------------------------------------------------------- */
#define PI_UART_BAUD            9600U   /* UART4: PA0 = TX (A0), PA1 = RX (A1) */
#define CMD_WATCHDOG_MS         500U    /* stop if no motor instruction for this
                                         * long while driving straight/reverse.
                                         * 0 = off */
#define ODOM_PERIOD_MS          200U    /* odometer message rate while moving
                                         * (= the Pi's nav CYCLE_TIME) */
#define ODOM_IDLE_PERIOD_MS     1000U   /* odometer rate while stopped or turning.
                                         * 0 = none */
#define ODOM_STOP_SETTLE_MS     150U    /* after a straight/reverse ends, wait this
                                         * long for the wheels to stop, then send
                                         * one final odometer. 0 = no final report */
#define BUTTON_HOLDOFF_MS       1000U   /* one button-press message per window */

/* ---- Debug (USB / ST-LINK virtual COM port, 115200) ----------------------- */
#define DEBUG_STATUS_MS         500U    /* status line while moving. 0 = off */
#define DEBUG_ECHO_PI           1       /* 1 = log every Pi message in and out */

#endif /* ROBOT_CONFIG_H */
