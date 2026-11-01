#ifndef USART_H
#define USART_H

#include <stdint.h>

/* USART1 serial console for _write() / miniprintf debug output.
 * Pins: PA_9 = USART1_TX (AF7), PA_10 = USART1_RX (AF7), APB2 clock.
 * Baud: 115200 (STDIO_BAUD from configuration.h).
 */

/* Initialise USART1 for polled transmit. */
void usart_init(void);

/* Transmit a single byte (polled, waits for TXE). */
void usart_putc(char c);

/* Transmit a buffer of bytes (polled). */
void usart_write(const char *ptr, int len);

#endif /* USART_H */
