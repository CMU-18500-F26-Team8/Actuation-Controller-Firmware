/**
 * @file    pi_link.h
 * @brief   Line-based CSV link to the Raspberry Pi on UART4 (9600 8N1),
 *          plus the same command input on the USB debug port (USART2).
 *
 * Every message, both directions, is one line ending in '\n':
 *     Message Type Code,Body 1,Body 2,Body 3,Body 4\n
 * Reception and transmission are interrupt-driven through ring buffers, so
 * nothing is lost while the main loop is busy and sending never blocks.
 *
 * Wiring: Nucleo A0 (PA0, UART4 TX) -> Pi GPIO15 RXD (pin 10)
 *         Nucleo A1 (PA1, UART4 RX) <- Pi GPIO14 TXD (pin 8)
 *         Nucleo GND               -- Pi GND (pin 6)
 */
#ifndef PI_LINK_H
#define PI_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define PI_FIELDS        5    /* type + 4 bodies */
#define PI_LINE_MAX      80   /* longer lines are dropped */

typedef enum { PI_SRC_PI = 0, PI_SRC_DEBUG = 1 } pi_src_t;

typedef struct {
  pi_src_t    src;
  uint8_t     nfields;              /* fields actually present (1..5) */
  const char *field[PI_FIELDS];     /* trimmed; missing ones point to "" */
} pi_msg_t;

void pi_link_init(void);

/** Returns true and fills `msg` with the next complete line, from either
 *  port. The strings stay valid until the next call. */
bool pi_link_get(pi_msg_t *msg);

/** Sends "type,b1,b2,b3,b4\n" to the Pi (NULL bodies are sent empty).
 *  Never blocks; returns false if the TX buffer was full (message dropped). */
bool pi_link_send(const char *type, const char *b1, const char *b2,
                  const char *b3, const char *b4);

/** Diagnostics: lines dropped because they were too long, RX overruns,
 *  TX messages dropped because the buffer was full. */
uint32_t pi_link_rx_dropped(void);
uint32_t pi_link_tx_dropped(void);

#endif /* PI_LINK_H */
