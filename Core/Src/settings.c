/**
 * @file    settings.c
 * @brief   Append-only settings records in flash sector 7. See settings.h.
 *
 * Record (4 words): magic, gyro scale (float bits; 0 = none), the
 * IMU_GYRO_SCALE it was saved under (float bits), check word. A record
 * whose check word doesn't match (e.g. power lost mid-write) is skipped.
 * A slot of four 0xFFFFFFFF words is free.
 */
#include "settings.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

#include <string.h>

#ifndef STORE_ADDR                        /* (a host test points it at RAM) */
#define STORE_ADDR      0x08060000UL      /* sector 7 */
#endif
#define STORE_SIZE      0x20000UL         /* 128 KB */
#define STORE_SECTOR    FLASH_SECTOR_7
#define RECORD_WORDS    4U
#define RECORD_BYTES    (RECORD_WORDS * 4U)
#define SLOTS           (STORE_SIZE / RECORD_BYTES)
#define MAGIC           0x53424731UL      /* "SBG1" */
#define FREE_WORD       0xFFFFFFFFUL

static bool     s_have;                   /* a valid record exists */
static uint32_t s_rec[RECORD_WORDS];      /* the newest valid record */
static uint32_t s_next_slot;              /* first slot after the last used one */

static inline uint32_t rotl(uint32_t v, unsigned n) { return (v << n) | (v >> (32U - n)); }

static uint32_t check_word(const uint32_t *w)
{
  return (w[0] ^ rotl(w[1], 7) ^ rotl(w[2], 13)) + 0x5A5A5A5AUL;
}

static uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static float    u2f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static const volatile uint32_t *slot_ptr(uint32_t i)
{
  return (const volatile uint32_t *)(STORE_ADDR + i * RECORD_BYTES);
}

static void flush_data_cache(void)
{
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

void settings_init(void)
{
  s_have = false;
  s_next_slot = 0U;
  for (uint32_t i = 0U; i < SLOTS; i++)
  {
    const volatile uint32_t *p = slot_ptr(i);
    uint32_t w[RECORD_WORDS] = { p[0], p[1], p[2], p[3] };
    bool free_slot = (w[0] & w[1] & w[2] & w[3]) == FREE_WORD;
    if (free_slot) continue;
    s_next_slot = i + 1U;                  /* never write below a used slot */
    if (w[0] == MAGIC && w[3] == check_word(w))
    {
      memcpy(s_rec, w, sizeof(s_rec));
      s_have = true;
    }
  }
}

bool settings_gyro_scale(float *out)
{
  if (!s_have || s_rec[1] == 0U) return false;
  if (s_rec[2] != f2u(IMU_GYRO_SCALE)) return false;   /* #define changed since */
  float v = u2f(s_rec[1]);
  if (!(v >= 0.5f && v <= 1.5f)) return false;
  *out = v;
  return true;
}

bool settings_gyro_scale_ignored(float *saved, float *saved_under)
{
  if (!s_have || s_rec[1] == 0U || s_rec[2] == f2u(IMU_GYRO_SCALE)) return false;
  *saved = u2f(s_rec[1]);
  *saved_under = u2f(s_rec[2]);
  return true;
}

static bool erase_store(void)
{
  FLASH_EraseInitTypeDef e = {0};
  uint32_t bad = 0U;
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Sector = STORE_SECTOR;
  e.NbSectors = 1U;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7-3.6 V: word programming */
  bool ok = HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
  flush_data_cache();
  return ok;
}

bool settings_save_gyro_scale(float scale)
{
  uint32_t w[RECORD_WORDS] = { MAGIC, (scale == 0.0f) ? 0U : f2u(scale), f2u(IMU_GYRO_SCALE), 0U };
  w[3] = check_word(w);

  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  bool ok = true;
  if (s_next_slot >= SLOTS)
  {
    ok = erase_store();
    s_next_slot = 0U;
  }
  uintptr_t addr = STORE_ADDR + s_next_slot * RECORD_BYTES;
  for (uint32_t k = 0U; ok && k < RECORD_WORDS; k++)
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4U * k, w[k]) == HAL_OK;
  (void)HAL_FLASH_Lock();
  flush_data_cache();
  s_next_slot++;                           /* used (even if the write failed) */

  const volatile uint32_t *p = (const volatile uint32_t *)addr;
  ok = ok && p[0] == w[0] && p[1] == w[1] && p[2] == w[2] && p[3] == w[3];
  if (ok)
  {
    memcpy(s_rec, w, sizeof(s_rec));
    s_have = true;
  }
  return ok;
}
