/*
 * usart.c — USART1 serial console driver for bare-metal firmware
 *
 * USART1 on APB2 (84 MHz), pins PA_9 (TX, AF7) / PA_10 (RX, AF7).
 * Baud 115200. Polled transmit only — no interrupts, no DMA.
 * Used to retarget _write() / miniprintf() so debug output reaches
 * a serial console (e.g. connected to Pi4 UART).
 */

#include "usart.h"
#include "configuration.h"
#include "gpio.h"

#include <stdint.h>

/* USART1 base (APB2) */
#define USART1_BASE 0x40011000

/* USART register offsets */
#define USART_SR  0x00
#define USART_DR  0x04
#define USART_BRR 0x08
#define USART_CR1 0x0C

/* USART_SR bits */
#define USART_SR_TXE  (1U << 7)  /* Transmit data register empty */
#define USART_SR_TC   (1U << 6)  /* Transmission complete */

/* USART_CR1 bits */
#define USART_CR1_UE  (1U << 13) /* USART enable */
#define USART_CR1_TE  (1U << 3)  /* Transmitter enable */
#define USART_CR1_RE  (1U << 2)  /* Receiver enable */

/* GPIO AF for USART1 on STM32F446: AF7 */
#define GPIO_AF_USART1 7

/* Baud rate register value for APB2=84MHz @ 115200 baud, 16x oversampling:
 * USARTDIV = 84000000 / 115200 = 729.1666...
 * Mantissa = 729 (0x2D9), Fraction = round(0.1666... * 16) = 3 (0x3)
 * BRR = (0x2D9 << 4) | 0x3 = 0x2D93
 */
#define USART1_BRR_115200_84MHZ 0x2D93

static volatile uint32_t *USART1_SR  = (volatile uint32_t *)(USART1_BASE + USART_SR);
static volatile uint32_t *USART1_DR  = (volatile uint32_t *)(USART1_BASE + USART_DR);
static volatile uint32_t *USART1_BRR = (volatile uint32_t *)(USART1_BASE + USART_BRR);
static volatile uint32_t *USART1_CR1 = (volatile uint32_t *)(USART1_BASE + USART_CR1);

void usart_init(void)
{
    /* Configure GPIO pins for USART1 alternate function */
    static GPIOPin gpio_tx, gpio_rx;
    gpio_init(&gpio_tx, "PA_9",  GPIO_ALTERNATE, GPIO_NONE);
    gpio_init(&gpio_rx, "PA_10", GPIO_ALTERNATE, GPIO_NONE);

    /* Set AF7 on PA_9 and PA_10 (GPIOA_AFRH, bits for pins 9 and 10) */
    volatile uint32_t *GPIOA_AFRH = (volatile uint32_t *)(0x40020000 + 0x24);
    *GPIOA_AFRH = (*GPIOA_AFRH & ~(0xF << ((9 - 8) * 4))) | (GPIO_AF_USART1 << ((9 - 8) * 4));
    *GPIOA_AFRH = (*GPIOA_AFRH & ~(0xF << ((10 - 8) * 4))) | (GPIO_AF_USART1 << ((10 - 8) * 4));

    /* Enable USART1 clock (APB2ENR bit 14) */
    volatile uint32_t *RCC_APB2ENR = (volatile uint32_t *)(0x40023800 + 0x44);
    *RCC_APB2ENR |= (1U << 14);

    /* Configure baud rate */
    *USART1_BRR = USART1_BRR_115200_84MHZ;

    /* Enable USART, transmitter, receiver */
    *USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void usart_putc(char c)
{
    /* Wait for TXE (transmit data register empty) */
    while (!(*USART1_SR & USART_SR_TXE)) { }
    *USART1_DR = (uint32_t)(uint8_t)c;
}

void usart_write(const char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        usart_putc(ptr[i]);
    }
}
