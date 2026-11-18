/**
 * @file    uart_log.c
 * @brief   USART2 TX on PA2 -> ST-LINK virtual COM port.
 */
#include "uart_log.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

void uart_log_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;   /* PA2 = TX, PA3 = RX */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  __HAL_RCC_USART2_CLK_ENABLE();
  USART2->CR1 = 0U;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  /* Oversampling by 16: BRR = fPCLK1 / baud (mantissa/fraction packed). */
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  USART2->BRR = (pclk1 + UART_LOG_BAUD / 2U) / UART_LOG_BAUD;
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_putc(char c)
{
  while ((USART2->SR & USART_SR_TXE) == 0U) { }
  USART2->DR = (uint8_t)c;
}

void uart_log(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;

  for (int i = 0; i < n; i++)
  {
    if (buf[i] == '\n') uart_putc('\r');   /* terminals want CRLF */
    uart_putc(buf[i]);
  }
}
