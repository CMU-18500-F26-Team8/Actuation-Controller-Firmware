/**
 * @file    uart_log.h
 * @brief   Blocking printf-style logging on USART2 (Nucleo ST-LINK virtual
 *          COM port, 115200 8N1). Register-level, no HAL UART needed.
 */
#ifndef UART_LOG_H
#define UART_LOG_H

#define UART_LOG_BAUD  115200U

void uart_log_init(void);
void uart_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* UART_LOG_H */
