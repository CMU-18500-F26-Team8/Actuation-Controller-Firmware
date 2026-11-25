# Motor controller: wiring, bench demo and Pi mode

Bare-metal bring-up for both SnackBot drive motors:
Nucleo-F446RE → Pololu #1213 Dual MC33926 carrier (channels M1 and M2) → 2× Pololu #4752 30:1 gearmotors with 64 CPR encoders.

Drive scheme: **PWM on IN1/IN2** (TIM1 CH1–CH4, 20 kHz), **D2 held HIGH**, D1 held LOW. During each PWM off-time the bridge brakes (slow decay), so speed is close to proportional to duty, and the SF fault flags are only LOW on real faults.

## Wiring

**Battery unplugged while wiring. Wheels off the table.**

| Nucleo | MCU | Function | Goes to |
|---|---|---|---|
| 3V3 (CN6) | — | logic supply | Driver **VDD** |
| GND (CN6) | — | ground | Driver logic **GND** |
| D9 | PC7 | GPIO out | **EN** (both channels) |
| D4 | PB5 | GPIO out, HIGH while running | **M1D2 + M2D2** (jumper the two rows together) |
| D6 | PB10 | GPIO out, LOW | **M1D1 + M2D1** (or bridge both D1 square GND pads and omit) |
| D7 | PA8 | TIM1 CH1 PWM | **M1IN1** |
| D8 | PA9 | TIM1 CH2 PWM | **M1IN2** |
| D2 | PA10 | TIM1 CH3 PWM | **M2IN1** |
| CN10 pin 14 (morpho) | PA11 | TIM1 CH4 PWM | **M2IN2** |
| D5 | PB4 | input, pull-up | **M1SF** |
| A3 | PB0 | input, pull-up | **M2SF** |
| 5V (via rail) | — | encoder supply | both encoders' **blue** |
| GND | — | ground | both encoders' **green** |
| D12 / D11 | PA6 / PA7 | TIM3 encoder | encoder 1 **yellow** / **white** |
| CN7 pin 17 (morpho) / D3 | PA15 / PB3 | TIM2 encoder | encoder 2 **yellow** / **white** (moved off D10, see below) |

Driver: **SLEW bridged to VDD**, INV open, battery + → VIN pad, − → GND pad, motor 1 red/black → M1OUT1/M1OUT2, motor 2 red/black → M2OUT1/M2OUT2. The VIN pin at the bottom of the logic header is live 12 V: keep its row empty.

All signal pins above are 5 V-tolerant (FT) per the STM32F446 datasheet.

## Run

1. Set `ROBOT_MODE 1` in `Core/Inc/robot_config.h` (bench demo), then **Build**, **Run and Debug** → Continue.
2. `screen /dev/tty.usbmodem* 115200`. You see the banner and `[idle]`.
3. Turn each wheel by hand: `M1 pos` / `M2 pos` change (1920 counts per wheel turn).
4. Connect 12 V, press **B1**. The demo repeats every 22 s:

| Phase | M1 | M2 |
|---|---|---|
| 1 | forward | forward |
| 2 | reverse | reverse |

Each phase ramps to `DEMO_MAX_DUTY` (20 %) over 0.25 s, holds 10 s, ramps down 0.25 s, pauses 0.5 s. Press **B1** again to stop.

```
[phase 1] both forward
t=    8.00s  M1 +20.0% +65.6rpm    M2 +20.0% +65.9rpm  sf=ok
```

20 % is about 65 rpm at the wheel (the 30:1 motor is ~330 rpm at 12 V). Both rpm columns should follow the sign of their duty.

"Forward" means positive duty. M2 is set up for mirror-image mounting: `MOTOR2_INVERT 1` in `motor.h` (positive duty = IN2 PWM). Encoder 2 counts up going forward with `ENC2_DIRECTION (-1)` in `encoder.h` (checked on the robot with the CN7-17 / D3 wiring). If a wheel's speed has the wrong sign, flip that encoder's `ENCx_DIRECTION`.

**Encoder 2 pins:** the D10 header pin stopped passing encoder 2's A signal, so encoder 2 now runs on TIM2: yellow (A) → **CN7 pin 17** (PA15, outer column, row 9; pin 15 above it is SWCLK, pin 19 below is GND), white (B) → **D3** (PB3). `ENC2_ALT_PINS 1` in `motor_pins.h` selects this; `0` goes back to D10 / CN7 pin 21 on TIM4. To check an encoder, type `telem,on` on the USB port and turn the wheel slowly by hand: that wheel's speed in the T lines (vL or vR) should follow it, positive going forward.

**Single-motor test:** set `DEMO_ONLY_MOTOR` in `motor_demo.h` to `1` (motor 1 only) or `2` (motor 2 only); `0` runs both. The other channel stays at 0 % duty, and its SF line is printed but ignored by the self-test verdict and the fault stop. 

**IN-pin wiring test:** set `DEMO_PIN_TEST 1` in `motor_demo.h`. B1 then keeps the driver asleep (motors can't move) and drives the four IN pins HIGH one at a time as steady DC, 5 s each, naming the pin in the log. Measure at the driver's solder joints (black probe on GND): only the named pin should read ~3.3 V. A pin that is HIGH in every step is open (the driver pulls unconnected IN pins HIGH).

## Pi control mode (default)

`ROBOT_MODE 0` in `Core/Inc/robot_config.h` (the default) runs the robot from the navigation Raspberry Pi. `ROBOT_MODE 1` runs the bench demo described above.

**Wiring (3 wires, both sides 3.3 V logic):**

| Pi | Nucleo |
|---|---|
| GPIO14 TXD (pin 8) | A1 (PA1, UART4 RX) |
| GPIO15 RXD (pin 10) | A0 (PA0, UART4 TX) |
| GND (pin 6) | GND |

**Power from the battery (no laptop):** 5 V / 1 A regulator fed from the 12 V battery. Regulator 5V OUT → **E5V (CN7 pin 6)**, regulator GND → Nucleo GND (CN7 pin 8), and move jumper **JP5 from U5V to E5V**. Power up the regulator first, check LD3 is lit, then plug in USB only if you want to flash or read the log. JP5 back to U5V returns to USB power.

**Protocol:** UART4, 9600 8N1. Every message, both directions, is `type,body1,body2,body3,body4` followed by `\n`.

| Pi → STM32 | Effect |
|---|---|
| `motor-instruction,straight,,,` / `reverse` | drive at `TARGET_SPEED_MPS` (0.16), per-wheel PID |
| `motor-instruction,stop,,,` | brake; also aborts a turn |
| `motor-instruction,turn,<deg>,,` | spin in place, + = counter-clockwise; other motor instructions are ignored until done |
| `odometer-reset,,,,` | odometer to 0 |
| `leds-on-off,true\|false,,,` | LD2 solid |
| `button-led-on-off,true\|false,,,` | LD2 1 Hz blink (alert LEDs take priority) |

| STM32 → Pi | When |
|---|---|
| `turn-complete,<deg>,,,` | turn finished (or aborted by stop); angle measured by the gyro, or by the encoders without one (see IMU below) |
| `odometer,<m>,,,` | every 0.2 s while driving straight/reverse, one final report 0.15 s after stopping, and every 1 s while stopped or turning; distance travelled, always increasing |
| `button-press,,,,` | B1 pressed; at most one per second |

A 500 ms watchdog stops the robot if motor instructions stop arriving while driving straight. The USB debug port (115200) logs every message and a status line with each wheel's setpoint, speed and duty, and accepts the same commands typed with Enter, for tuning without the Pi.

**Live PID tuning:** `pip3 install pyserial matplotlib`, close any other serial monitor, then `python3 tools/pid_tune.py` (`--demo` runs it on simulated motors). It turns on 50 Hz telemetry, plots setpoint / speed / duty per wheel, sends gains as you type them, and runs timed Forward / Reverse tests with rise time, overshoot, settle time, steady error and the duty each wheel needed. Each wheel has its own gains (left column / right column); untick **show L** or **show R** to look at one wheel. USB-only commands it uses: `get`, `set,<kp_l|ki_l|kd_l|kp_r|ki_r|kd_r|speed|turn|accel>,<value>`, `telem,on|off`. Changes last until reset; **Print config** prints the `#define` lines to paste back.

Each wheel runs a plain PID, `duty = KP·err + KI·∫err + KD·d(err)/dt`, with no feed-forward and no left/right coupling. Order, per wheel: wheels in the air, `accel` 100 (a true step) → `ki` 0, `kd` 0, raise `kp` until it rings, halve it (it settles below target with P alone) → raise `ki` until it reaches target in ~0.5 s with <10 % overshoot (`kd` stays 0) → restore `accel` 0.4 → repeat on the floor and keep those gains.

**Sides:** M2 is the left wheel and M1 the right (`LEFT_MOTOR` / `RIGHT_MOTOR` in `robot_config.h`), so the tuner's `_l` gains act on M2 and `_r` on M1.

**Tuning:** every gain, speed and geometry value is in `robot_config.h` (`PID_L_KP/KI/KD`, `PID_R_KP/KI/KD`, `TARGET_SPEED_MPS`, `TURN_SPEED_MPS`, `TRACK_WIDTH_M`, ...). Code: `pi_link.c` (UART), `drive.c` (PID, odometry, turns), `imu.c` (gyro), `robot_app.c` (messages, button, LED).

## IMU: gyro-measured turns

A HiLetgo HW-123 (GY-521 board, MPU-6050) gives the yaw rate. In Pi mode, a turn ends when the **gyro** has turned the commanded angle, the robot brakes and settles for 150 ms (`TURN_SETTLE_MS`), and `turn-complete` reports the gyro's angle. Wheel slip in an in-place turn no longer goes into the heading. The Pi protocol is unchanged.

**Wiring (4 wires):**

| HW-123 | Nucleo |
|---|---|
| VCC | 3V3 (or 5V: the board has its own regulator) |
| GND | GND |
| SCL | **D15** (PB8, I2C1 SCL) |
| SDA | **D14** (PB9, I2C1 SDA) |
| AD0, INT, XDA, XCL | leave open (address 0x68) |

**3V3 sits right next to RESET**: CN6 pin 3 (RESET) / pin 4 (3V3), and CN7 pin 14 (RESET) / pin 16 (3V3). VCC one pin off, on RESET, loads NRST down: the board stops running and the debugger fails with `DEV_TARGET_HELD_UNDER_RESET` ("could not connect"). The HW-123's power LED lights only when VCC is really on 3V3.

Mount it flat, chip side up, on something rigid (foam tape against motor vibration), as close to the midpoint between the wheels as is easy. Position doesn't change the yaw rate, but flat does.

**Fallbacks.** Without a working IMU, turns run on the encoders exactly as before:
- **Not found at boot:** `[imu] no answer ...`, and the firmware re-probes it every 0.5 s.
- **3 reads fail in a row:** the IMU is marked down, and a turn under way finishes on the encoders.
- **The gyro turns the opposite way to the wheels** (a wrong `IMU_YAW_SIGN`): the encoders finish the turn.
- **The wheels get far ahead of the gyro** (more than 1.5 × the angle + 30°; the robot is stuck or spinning its wheels): the turn stops and reports the angle the gyro saw (`TURN_SLIP_*`).

**Bias.** The gyro reads slightly off zero at rest, and this offset drifts with temperature. The firmware learns it while the robot stands still: stopped and wheels at rest for 0.3 s, then 0.5 s windows (`IMU_STILL_AFTER_MS`, `IMU_BIAS_WINDOW`). A window in which the readings jump (someone bumped the robot) is thrown away. After power-up, keep the robot still for about 1 s before the first turn. Until a bias is learnt, turns use the encoders.

**Bring-up** (USB debug port):

1. Boot and look for `[imu] MPU-6050 at 0x68, WHO_AM_I 0x68`. A different WHO_AM_I (0x70, 0x72, 0x98) is a clone chip and usually works.
2. About 1 s later you should see `[imu] gyro bias learnt: ... deg/s`. Then type `imu`: `present`, rate near 0.00, and errors not counting up.
3. **Sign:** type `imu,watch,on` and rotate the robot counter-clockwise by hand (seen from above). The yaw must go **up**. If it goes down, set `IMU_YAW_SIGN (-1)`. Type `imu,watch,off` when done.
4. **Scale:** line the robot up against a mark, type `imu,zero`, turn it slowly by hand 5 whole turns counter-clockwise back onto the mark, then type `imu`. Compute scale = 1800 / yaw. You can try it live with `imu,scale,<v>`, then copy it into `IMU_GYRO_SCALE`. Expect a value within a few percent of 1.000.
5. **Drive test:** send `motor-instruction,turn,90,,` and read the `[turn]` line. It shows the target, the gyro's angle, the encoders' angle and which one was reported. The difference between gyro and encoders is the slip the encoders used to miss.

USB-only IMU commands: `imu` (status), `imu,zero`, `imu,cal` (forget the bias and learn it again), `imu,scale,<v>`, `imu,watch,on|off`. `IMU_TURNS 0` keeps the IMU running (status, watch) but turns go back to the encoders. `IMU_ENABLED 0` removes all IMU code.

## LED / button

| LD2 | Meaning |
|---|---|
| Slow blink | Idle, driver asleep (wheels coast) |
| Solid | Running |
| Fast blink | Fault: both motors stopped. Press B1 to clear (drops EN, resets the MC33926) |

## Faults

**Start-up self-test** (each B1 start, before any PWM, 2 × 20 ms):

1. EN HIGH, D2 LOW. Both SF lines must read **LOW** (the MC33926 reports "disabled"). If one reads HIGH, that SF wire is open or on the wrong pin, EN isn't reaching the driver, or there is no VIN. Warning only; the run continues. Expect this warning with the battery unplugged.
2. D2 HIGH, 0 % duty. Both SF lines must read **HIGH**. If one stays LOW, the cause is not PWM or motor current: check that channel's D2/D1 at the driver pin, VIN, or an OUT pin shorted to VIN. With `DEMO_STOP_ON_FAULT 1` the run does not start.

Each line also prints the EN/D1/D2 levels as read back at the MCU pins.

**While running**, SF is sampled once per millisecond. SF LOW on 3 consecutive samples on either channel latches a fault and prints one `[sf]` line with the time since the run started, the phase, both duties and the other channel's SF. With `DEMO_STOP_ON_FAULT 1` this is followed by `[FAULT] … on M1/M2` and both motors stop. Causes: short/over-current or over-temperature (latched in the chip until B1), VIN under-voltage (clears itself). Telemetry shows `sf=ok` or which line is LOW right now.

Reading the result: self-test 2/2 fails → wiring/VIN, motors not involved. Self-test passes and `[sf]` appears later → output short, over-current or a VIN dip under load; the duty and phase in the `[sf]` line say which direction was being driven.

## Troubleshooting

| Symptom | Check |
|---|---|
| A motor runs full speed at 0 % and won't reverse, no fault | One IN pin isn't reaching the chip (open joint; the driver pulls it HIGH). Reflow that IN joint; `DEMO_PIN_TEST` |
| A motor doesn't turn, no fault | D4 → both **D2** pins HIGH while running; D6 → both **D1** LOW; EN HIGH |
| One motor spins backwards vs the other | Normal if wired differently; swap that motor's red/black or set `MOTORx_INVERT` |
| `rpm` sign opposite to duty | Swap that encoder's yellow/white or flip `ENCx_DIRECTION` |
| Encoder 2 reads 0, or its count wobbles 0 / 65535 (one channel dead) | `telem,on` and turn the wheel by hand: a dead channel leaves its speed stuck near 0. Check **CN7 pin 17** (A) and **D3** (B); female jumpers on morpho pins |
| `[FAULT]` immediately | 12 V on VIN? Both D2 pins really HIGH (D2 LOW also pulls SF LOW)? |
| Breakpoint hit while running | Both motors brake: TIM1 freezes and forces all IN pins LOW |

## Code layout

| File | Role |
|---|---|
| `Core/Inc/motor_pins.h` | Every pin assignment |
| `Core/Src/motor.c` | TIM1 4-channel PWM, EN/D1/D2, SF debounce (register-level) |
| `Core/Src/encoder.c` | TIM3 + TIM4 quadrature decoders |
| `Core/Src/uart_log.c` | Blocking log on USART2 |
| `Core/Src/motor_demo.c` | Bench demo (`ROBOT_MODE 1`): self-test, straight/spin moves, telemetry |
| `Core/Inc/robot_config.h` | Pi-mode tunables: speeds, PID gains, geometry, timing |
| `Core/Src/pi_link.c` | UART4 link to the Pi (+ typed commands on USB), ring buffers |
| `Core/Src/drive.c` | One speed PID per wheel (own gains), odometry, in-place turns (gyro-ended, encoder fallback) |
| `Core/Src/imu.c` | MPU-6050 Z gyro on I2C1 (register-level): yaw, bias learnt at rest, re-probe on failure |
| `Core/Src/robot_app.c` | Pi messages → drive, odometer/turn/button reports, LD2 |
| `Core/Src/main.c` | `robot_app_init` / `robot_app_poll` in `USER CODE` blocks |

Peripherals are configured in code, not CubeMX, so regenerating from the `.ioc` doesn't touch them.
