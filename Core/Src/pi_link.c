/**
 * @file    pi_link.c
 * @brief   UART4 (Pi) RX/TX and USART2 (debug) RX, interrupt-driven ring
 *          buffers, '\n'-terminated CSV lines. Register-level, no HAL UART.
 */
#include "pi_link.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

#define RING_SIZE   256U                     /* power of two */
#define RING_MASK   (RING_SIZE - 1U)

typedef struct {
  volatile uint16_t head;                    /* written by the producer */
  volatile uint16_t tail;                    /* written by the consumer */
  uint8_t           buf[RING_SIZE];
} ring_t;

typedef struct {
  char     buf[PI_LINE_MAX + 1];
  uint16_t len;
  bool     overflow;
} line_t;

static ring_t   s_pi_rx, s_pi_tx, s_dbg_rx;
static line_t   s_line[2];                   /* indexed by pi_src_t */
static char     s_parsed[PI_LINE_MAX + 1];   /* storage behind pi_msg_t */
static volatile uint32_t s_rx_dropped;
static volatile uint32_t s_tx_dropped;

/* ---- ring helpers --------------------------------------------------------- */

static inline uint16_t ring_used(const ring_t *r)
{
  return (uint16_t)((r->head - r->tail) & RING_MASK);
}

static inline bool ring_push(ring_t *r, uint8_t c)
{
  uint16_t next = (uint16_t)((r->head + 1U) & RING_MASK);
  if (next == r->tail) return false;         /* full (one slot kept free) */
  r->buf[r->head] = c;
  r->head = next;
  return true;
}

static inline bool ring_pop(ring_t *r, uint8_t *c)
{
  if (r->head == r->tail) return false;
  *c = r->buf[r->tail];
  r->tail = (uint16_t)((r->tail + 1U) & RING_MASK);
  return true;
}

/* ---- init ------------------------------------------------------------------ */

void pi_link_init(void)
{
  GPIO_InitTypeDef g = {0};

  memset(s_line, 0, sizeof(s_line));
  s_pi_rx.head = s_pi_rx.tail = 0U;
  s_pi_tx.head = s_pi_tx.tail = 0U;
  s_dbg_rx.head = s_dbg_rx.tail = 0U;

  /* UART4: PA0 = TX, PA1 = RX (AF8). Pull-up keeps an unplugged RX idle. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &g);

  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_UART4_FORCE_RESET();
  __HAL_RCC_UART4_RELEASE_RESET();
  UART4->CR1 = 0U;
  UART4->CR2 = 0U;                           /* 1 stop bit */
  UART4->CR3 = 0U;
  /* Oversampling by 16: BRR = fPCLK1 / baud (UART4 is on APB1). */
  UART4->BRR = (HAL_RCC_GetPCLK1Freq() + PI_UART_BAUD / 2U) / PI_UART_BAUD;
  UART4->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  /* USART2 is set up by uart_log_init() (TX polled); add the RX interrupt
   * so commands can also be typed on the USB debug port. */
  USART2->CR1 |= USART_CR1_RXNEIE;

  HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ---- interrupts ------------------------------------------------------------ */

void UART4_IRQHandler(void)
{
  uint32_t sr = UART4->SR;

  /* Reading SR then DR clears RXNE and the ORE/NE/FE error flags. */
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE))
  {
    uint8_t c = (uint8_t)UART4->DR;
    if ((sr & USART_SR_ORE) || !ring_push(&s_pi_rx, c)) s_rx_dropped++;
  }

  if ((UART4->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
  {
    uint8_t c;
    if (ring_pop(&s_pi_tx, &c)) UART4->DR = c;
    else                        UART4->CR1 &= ~USART_CR1_TXEIE;
  }
}

void USART2_IRQHandler(void)
{
  uint32_t sr = USART2->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE))
  {
    uint8_t c = (uint8_t)USART2->DR;
    if ((sr & USART_SR_ORE) || !ring_push(&s_dbg_rx, c)) s_rx_dropped++;
  }
}

/* ---- line assembly + parsing ----------------------------------------------- */

static char *trim(char *s)
{
  while (*s == ' ' || *s == '\t') s++;
  char *e = s + strlen(s);
  while (e > s && (e[-1] == ' ' || e[-1] == '\t')) *--e = '\0';
  return s;
}

/** Splits s_parsed into up to PI_FIELDS comma-separated fields. */
static void parse(pi_msg_t *msg, pi_src_t src)
{
  static const char empty[] = "";
  char *p = s_parsed;

  msg->src = src;
  msg->nfields = 0U;
  for (uint8_t i = 0; i < PI_FIELDS; i++) msg->field[i] = empty;

  while (msg->nfields < PI_FIELDS)
  {
    char *comma = strchr(p, ',');
    if (comma) *comma = '\0';
    msg->field[msg->nfields++] = trim(p);
    if (!comma) break;
    p = comma + 1;                         /* fields past the 5th are ignored */
  }
}

/** Feeds characters from one ring into its line buffer. Returns true when a
 *  complete, non-empty line has been copied into s_parsed. */
static bool assemble(ring_t *r, line_t *l)
{
  uint8_t c;
  while (ring_pop(r, &c))
  {
    if (c == '\n' || c == '\r')            /* '\r' too, for typed debug input */
    {
      bool ok = !l->overflow && l->len > 0U;
      if (l->overflow) s_rx_dropped++;
      l->buf[l->len] = '\0';
      if (ok) memcpy(s_parsed, l->buf, (size_t)l->len + 1U);
      l->len = 0U;
      l->overflow = false;
      if (ok) return true;
      continue;
    }
    if (l->len < PI_LINE_MAX) l->buf[l->len++] = (char)c;
    else                      l->overflow = true;
  }
  return false;
}

bool pi_link_get(pi_msg_t *msg)
{
  if (assemble(&s_pi_rx, &s_line[PI_SRC_PI]))       { parse(msg, PI_SRC_PI);    return true; }
  if (assemble(&s_dbg_rx, &s_line[PI_SRC_DEBUG]))   { parse(msg, PI_SRC_DEBUG); return true; }
  return false;
}

/* ---- transmit -------------------------------------------------------------- */

bool pi_link_send(const char *type, const char *b1, const char *b2,
                  const char *b3, const char *b4)
{
  char out[PI_LINE_MAX + 2];
  int n = snprintf(out, sizeof(out), "%s,%s,%s,%s,%s\n", type,
                   b1 ? b1 : "", b2 ? b2 : "", b3 ? b3 : "", b4 ? b4 : "");
  if (n <= 0 || n >= (int)sizeof(out)) { s_tx_dropped++; return false; }

  /* All or nothing: never send half a message. */
  if ((uint16_t)(RING_SIZE - 1U - ring_used(&s_pi_tx)) < (uint16_t)n)
  {
    s_tx_dropped++;
    return false;
  }
  for (int i = 0; i < n; i++) (void)ring_push(&s_pi_tx, (uint8_t)out[i]);
  UART4->CR1 |= USART_CR1_TXEIE;           /* ISR drains the ring */
  return true;
}

uint32_t pi_link_rx_dropped(void) { return s_rx_dropped; }
uint32_t pi_link_tx_dropped(void) { return s_tx_dropped; }
