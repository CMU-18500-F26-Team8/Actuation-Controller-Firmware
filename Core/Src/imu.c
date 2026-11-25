/**
 * @file    imu.c
 * @brief   MPU-6050 Z gyro on I2C1 (PB8 = SCL, PB9 = SDA): yaw rate, yaw
 *          angle, and a bias learnt while the robot stands still.
 *
 * Timing uses the Cortex-M4 cycle counter (DWT->CYCCNT), so each sample is
 * integrated over its true interval (trapezoid rule), not a 1 ms tick.
 *
 * Bias: while the caller reports `still`, samples are collected in windows
 * of IMU_BIAS_WINDOW. A window whose spread (max - min) stays under
 * IMU_STILL_SPREAD_DPS (nobody bumped the robot) gives a mean; the first
 * mean becomes the bias, later ones are blended in by IMU_BIAS_BLEND. A
 * window that `still` interrupts is thrown away.
 *
 * I2C1 is driven at register level (polling, every wait bounded by
 * I2C_WAIT_US), like uart_log.c / motor.c / encoder.c: the CubeMX project
 * has no I2C HAL, and regenerating from the .ioc does not touch any of it.
 * Reads follow RM0390's 1-byte and 2-byte master-receiver sequences.
 */
#include "imu.h"
#include "robot_config.h"
#include "uart_log.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#if IMU_ENABLED

/* ---- MPU-6050 registers --------------------------------------------------- */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_GYRO_ZOUT_H   0x47
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

#define SMPLRT_DIV_1KHZ   0x00    /* 1 kHz / (1 + 0) with the low-pass on */
#define CONFIG_DLPF_42HZ  0x03    /* gyro low-pass 42 Hz, 4.8 ms delay */
#define GYRO_FS_250       0x00    /* +-250 deg/s: 131 LSB per deg/s */
#define LSB_PER_DPS       131.0f
#define PWR_RESET         0x80
#define PWR_CLK_PLL_XGYRO 0x01    /* wake, clock from the X gyro's PLL */

#define I2C_WAIT_US       1000U   /* longest wait for any one I2C event */
#define WARMUP_MS         100U    /* gyro start-up + low-pass settle */
#define FAILS_TO_DOWN     3U      /* consecutive read failures that mark it down */

/* ---- pins ------------------------------------------------------------------- */
#define I2C_PORT          GPIOB
#define SCL_PIN           GPIO_PIN_8   /* D15 */
#define SDA_PIN           GPIO_PIN_9   /* D14 */

static uint8_t  s_addr;                /* 8-bit bus address (7-bit << 1) */

static imu_status_t s_st;
static bool     s_fresh;               /* s_last_cyc is a real sample */
static uint32_t s_last_cyc;            /* DWT time of the last sample */
static uint32_t s_last_ok_ms;          /* HAL tick of the last good read */
static uint32_t s_warm_until_ms;
static uint32_t s_last_retry_ms;
static uint8_t  s_fail_run;
static float    s_prev_rate;           /* for the trapezoid */

/* bias window */
static uint16_t s_win_n;
static float    s_win_sum, s_win_min, s_win_max;

float g_imu_gyro_scale = IMU_GYRO_SCALE;

/* ---- cycle counter ------------------------------------------------------------ */

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) { }
}

/* ---- I2C set-up and bus recovery ----------------------------------------------- */

static void pins_af(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = SCL_PIN | SDA_PIN;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_PULLUP;                /* the board has its own pull-ups too */
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(I2C_PORT, &g);
}

/** A slave stuck mid-byte (e.g. reset during a read) holds SDA low forever.
 *  Clock SCL by hand until it lets go, then send a STOP. */
static void bus_recover(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = SCL_PIN | SDA_PIN;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(I2C_PORT, &g);
  delay_us(10);

  for (int i = 0; i < 9 && HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN) == GPIO_PIN_RESET; i++)
  {
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); delay_us(5);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);   delay_us(5);
  }
  /* STOP: SDA low -> high while SCL is high */
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET); delay_us(5);
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); delay_us(5);
  HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);   delay_us(5);
  HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);   delay_us(5);
}

/** Reset I2C1, free the bus, and set it up as a master at IMU_I2C_HZ. */
static bool i2c_start(void)
{
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  bus_recover();
  pins_af();

  uint32_t pclk = HAL_RCC_GetPCLK1Freq();          /* 42 MHz */
  uint32_t mhz  = pclk / 1000000U;
  I2C1->CR1 = I2C_CR1_SWRST;
  I2C1->CR1 = 0U;
  I2C1->CR2 = mhz & I2C_CR2_FREQ;
  if (IMU_I2C_HZ > 100000U)
  {
    uint32_t ccr = pclk / (3U * IMU_I2C_HZ);       /* fast mode, Tlow/Thigh = 2 */
    if (ccr < 1U) ccr = 1U;
    I2C1->CCR = I2C_CCR_FS | (ccr & I2C_CCR_CCR);
    I2C1->TRISE = (mhz * 300U) / 1000U + 1U;       /* 300 ns max rise */
  }
  else
  {
    uint32_t ccr = pclk / (2U * IMU_I2C_HZ);
    if (ccr < 4U) ccr = 4U;
    I2C1->CCR = ccr & I2C_CCR_CCR;
    I2C1->TRISE = mhz + 1U;                        /* 1000 ns max rise */
  }
  I2C1->CR1 = I2C_CR1_PE;
  return (I2C1->SR2 & I2C_SR2_BUSY) == 0U;
}

/** Wait for SR1 `flag`. Fails on timeout or on a NACK / bus error. */
static bool wait_sr1(uint32_t flag)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t limit = I2C_WAIT_US * (SystemCoreClock / 1000000U);
  for (;;)
  {
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & flag) return true;
    if (sr1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO)) return false;
    if ((DWT->CYCCNT - start) >= limit) return false;
  }
}

static bool wait_not_busy(void)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t limit = I2C_WAIT_US * (SystemCoreClock / 1000000U);
  while (I2C1->SR2 & I2C_SR2_BUSY)
    if ((DWT->CYCCNT - start) >= limit) return false;
  return true;
}

/** End a failed transaction: STOP, clear error flags, ACK/POS off. */
static bool fail(void)
{
  I2C1->CR1 |= I2C_CR1_STOP;
  I2C1->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO);
  I2C1->CR1 &= ~(I2C_CR1_ACK | I2C_CR1_POS);
  s_st.errors++;
  return false;
}

/** START (or repeated START) + address byte; ADDR left set for the caller. */
static bool start_addr(uint8_t addr_rw)
{
  I2C1->CR1 |= I2C_CR1_START;
  if (!wait_sr1(I2C_SR1_SB)) return false;
  I2C1->DR = addr_rw;
  return wait_sr1(I2C_SR1_ADDR);
}

static inline void clear_addr(void)
{
  (void)I2C1->SR1;
  (void)I2C1->SR2;
}

/** START, address + write, register pointer. Leaves the bus held. */
static bool select_reg(uint8_t reg)
{
  if (!wait_not_busy()) return false;
  if (!start_addr(s_addr)) return false;
  clear_addr();
  if (!wait_sr1(I2C_SR1_TXE)) return false;
  I2C1->DR = reg;
  return wait_sr1(I2C_SR1_BTF);
}

static bool reg_write(uint8_t reg, uint8_t v)
{
  if (!select_reg(reg)) return fail();
  I2C1->DR = v;
  if (!wait_sr1(I2C_SR1_BTF)) return fail();
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

/** Read n = 1 or 2 bytes starting at reg (the only sizes used here). */
static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
  if (n < 1U || n > 2U) return false;
  if (!select_reg(reg)) return fail();

  if (n == 1U)
  {
    I2C1->CR1 &= ~(I2C_CR1_ACK | I2C_CR1_POS);
    if (!start_addr((uint8_t)(s_addr | 1U))) return fail();
    __disable_irq();                     /* ADDR clear and STOP back to back */
    clear_addr();
    I2C1->CR1 |= I2C_CR1_STOP;
    __enable_irq();
    if (!wait_sr1(I2C_SR1_RXNE)) return fail();
    buf[0] = (uint8_t)I2C1->DR;
  }
  else
  {
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;
    if (!start_addr((uint8_t)(s_addr | 1U))) return fail();
    __disable_irq();                     /* ADDR clear and ACK off back to back */
    clear_addr();
    I2C1->CR1 &= ~I2C_CR1_ACK;
    __enable_irq();
    if (!wait_sr1(I2C_SR1_BTF)) return fail();   /* both bytes in DR + shift reg */
    I2C1->CR1 |= I2C_CR1_STOP;
    buf[0] = (uint8_t)I2C1->DR;
    buf[1] = (uint8_t)I2C1->DR;
    I2C1->CR1 &= ~I2C_CR1_POS;
  }
  return true;
}

/** Probe and configure. `reset`: full device reset first (blocks 100 ms;
 *  only at boot). Returns true if the chip answered and took the settings. */
static bool configure(bool reset)
{
  uint8_t who = 0U;
  if (!reg_read(REG_WHO_AM_I, &who, 1U)) return false;
  s_st.who_am_i = who;

  if (reset)
  {
    if (!reg_write(REG_PWR_MGMT_1, PWR_RESET)) return false;
    HAL_Delay(100);
  }
  if (!reg_write(REG_PWR_MGMT_1, PWR_CLK_PLL_XGYRO)) return false;
  if (!reg_write(REG_PWR_MGMT_2, 0x00))              return false;
  if (!reg_write(REG_SMPLRT_DIV, SMPLRT_DIV_1KHZ))   return false;
  if (!reg_write(REG_CONFIG, CONFIG_DLPF_42HZ))      return false;
  if (!reg_write(REG_GYRO_CONFIG, GYRO_FS_250))      return false;

  uint8_t chk[2] = {0xFF, 0xFF};       /* CONFIG, GYRO_CONFIG */
  if (!reg_read(REG_CONFIG, chk, 2U)) return false;
  if ((chk[0] & 0x07U) != CONFIG_DLPF_42HZ || (chk[1] & 0x18U) != GYRO_FS_250) return false;

  uint32_t now = HAL_GetTick();
  s_warm_until_ms = now + WARMUP_MS;
  s_fresh = false;
  s_fail_run = 0U;
  s_win_n = 0U;
  s_st.present = true;
  return true;
}

static void mark_down(void)
{
  s_st.present = false;
  s_fresh = false;
  s_win_n = 0U;
  s_last_retry_ms = HAL_GetTick();
}

/* ---- bias ------------------------------------------------------------------------ */

static void bias_window(float raw, bool still)
{
  if (!still) { s_win_n = 0U; return; }
  if (s_win_n == 0U) { s_win_sum = 0.0f; s_win_min = raw; s_win_max = raw; }
  s_win_sum += raw;
  if (raw < s_win_min) s_win_min = raw;
  if (raw > s_win_max) s_win_max = raw;
  if (++s_win_n < IMU_BIAS_WINDOW) return;

  s_win_n = 0U;
  if ((s_win_max - s_win_min) > IMU_STILL_SPREAD_DPS) return;   /* bumped */
  float mean = s_win_sum / (float)IMU_BIAS_WINDOW;
  if (!s_st.calibrated)
  {
    s_st.bias_dps = mean;
    s_st.calibrated = true;
    char b[16];
    long m = lrintf(mean * 1000.0f);
    snprintf(b, sizeof(b), "%s%ld.%03ld", (m < 0) ? "-" : "", labs(m) / 1000, labs(m) % 1000);
    uart_log("[imu] gyro bias learnt: %s deg/s\n", b);
  }
  else
  {
    s_st.bias_dps += IMU_BIAS_BLEND * (mean - s_st.bias_dps);
  }
  s_st.bias_updates++;
}

/* ---- public ------------------------------------------------------------------------ */

void imu_init(void)
{
  dwt_init();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  s_addr = (uint8_t)(IMU_I2C_ADDR << 1);

  bool bus = i2c_start();
  bool ok = bus && configure(true);
  if (ok)
  {
    uart_log("[imu] MPU-6050 at 0x%02X, WHO_AM_I 0x%02X%s. Gyro +-250 deg/s, 42 Hz low-pass. "
             "Bias is learnt while the robot stands still.\n",
             (unsigned)IMU_I2C_ADDR, (unsigned)s_st.who_am_i,
             (s_st.who_am_i == 0x68U) ? "" : " (not 0x68: a clone, usually fine)");
  }
  else
  {
    mark_down();
    uart_log("[imu] no answer at 0x%02X on I2C1 (D15 = SCL, D14 = SDA)%s: turns use the encoders. "
             "Retrying every %u ms.\n", (unsigned)IMU_I2C_ADDR, bus ? "" : ", I2C init failed",
             (unsigned)IMU_RETRY_MS);
  }
}

void imu_poll(bool still)
{
  uint32_t now_ms = HAL_GetTick();

  if (!s_st.present)
  {
    if ((now_ms - s_last_retry_ms) < IMU_RETRY_MS) return;
    s_last_retry_ms = now_ms;
    if (i2c_start() && configure(false))
      uart_log("[imu] back (WHO_AM_I 0x%02X)\n", (unsigned)s_st.who_am_i);
    else
      s_st.present = false;
    return;
  }

  uint32_t cyc = DWT->CYCCNT;
  uint32_t cyc_per_us = SystemCoreClock / 1000000U;
  if (s_fresh && (cyc - s_last_cyc) < IMU_SAMPLE_US * cyc_per_us) return;

  uint8_t b[2];
  if (!reg_read(REG_GYRO_ZOUT_H, b, 2U))
  {
    if (++s_fail_run >= FAILS_TO_DOWN)
    {
      mark_down();
      uart_log("[imu] %u reads failed in a row: down, turns use the encoders\n", (unsigned)FAILS_TO_DOWN);
    }
    return;
  }
  s_fail_run = 0U;
  s_st.samples++;
  s_last_ok_ms = now_ms;

  float raw = (float)(int16_t)(((uint16_t)b[0] << 8) | b[1]) / LSB_PER_DPS;
  s_st.raw_dps = raw;
  if ((int32_t)(now_ms - s_warm_until_ms) < 0) { s_fresh = false; return; }

  bias_window(raw, still);

  float rate = (float)IMU_YAW_SIGN * g_imu_gyro_scale * (raw - s_st.bias_dps);
  if (s_fresh && s_st.calibrated)
  {
    float dt = (float)(cyc - s_last_cyc) / (float)SystemCoreClock;
    s_st.yaw_deg += 0.5 * (double)(rate + s_prev_rate) * (double)dt;
  }
  s_st.rate_dps = rate;
  s_prev_rate = rate;
  s_last_cyc = cyc;
  s_fresh = true;
}

bool imu_ok(void)
{
  return s_st.present && s_st.calibrated && s_fresh
      && (HAL_GetTick() - s_last_ok_ms) <= IMU_STALE_MS;
}

double imu_yaw_deg(void) { return s_st.yaw_deg; }

void imu_zero(void) { s_st.yaw_deg = 0.0; }

void imu_recalibrate(void)
{
  s_st.calibrated = false;
  s_win_n = 0U;
}

void imu_get_status(imu_status_t *out) { *out = s_st; }

#else  /* IMU_ENABLED == 0: stubs, so callers need no #if */

float g_imu_gyro_scale = IMU_GYRO_SCALE;
void   imu_init(void) { }
void   imu_poll(bool still) { (void)still; }
bool   imu_ok(void) { return false; }
double imu_yaw_deg(void) { return 0.0; }
void   imu_zero(void) { }
void   imu_recalibrate(void) { }
void   imu_get_status(imu_status_t *out) { *out = (imu_status_t){0}; }

#endif /* IMU_ENABLED */
