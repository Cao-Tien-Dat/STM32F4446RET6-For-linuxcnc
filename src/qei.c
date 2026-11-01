/*
 * qei.c — Quadrature encoder interface module (hardware TIM2 encoder mode)
 * Port of Mbed QEI/QEIdriver (FirmwareSource/Remora-OS5/qei.h/cpp)
 *
 * Uses TIM2 in encoder mode (TI1FP1/TI2FP2) for hardware quadrature decoding:
 *   PA_0 = TIM2_CH1 (encoder phase A)
 *   PA_1 = TIM2_CH2 (encoder phase B)
 * The TIM2 hardware counter tracks position automatically on every encoder
 * edge; qei_update() simply reads TIM2->CNT and reports it via
 * txData.processVariable[0].
 *
 * Runs in the Base thread (TIMER_BASE, 120 kHz).
 */

#include "qei.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

/* TIM2 base (APB1, 42 MHz) */
#define TIM2_BASE 0x40000000

/* TIM2 register offsets */
#define TIM_CR1   0x00
#define TIM_SMCR  0x08
#define TIM_CCMR1 0x18
#define TIM_CCER  0x20
#define TIM_CNT   0x24
#define TIM_PSC   0x28
#define TIM_ARR   0x2C
#define TIM_EGR   0x14
#define TIM_SR    0x10

/* TIM_CR1 bits */
#define TIM_CR1_CEN   (1U << 0)

/* TIM_SMCR bits: SMS[2:0] at bits [2:0] */
#define TIM_SMCR_SMS_ENCODER_TI12  0x1  /* 001 = Encoder mode TI1FP1/TI2FP2 */

/* TIM_CCMR1 bits (input capture mode) */
#define TIM_CCMR1_CC1S_POS   0   /* CC1S bits [1:0] */
#define TIM_CCMR1_CC1S_INPUT (0x1 << TIM_CCMR1_CC1S_POS)   /* 01 = IC1 on TI1 */
#define TIM_CCMR1_CC2S_POS   8   /* CC2S bits [9:8] */
#define TIM_CCMR1_CC2S_INPUT (0x1 << TIM_CCMR1_CC2S_POS)   /* 01 = IC2 on TI2 */

/* TIM_CCER bits */
#define TIM_CCER_CC1E  (1U << 0)  /* Capture/Compare 1 enable */
#define TIM_CCER_CC2E  (1U << 4)  /* Capture/Compare 2 enable */

/* GPIO AF for TIM2 on STM32F446: AF1 */
#define GPIO_AF_TIM2 1

typedef struct {
    volatile float *ptr_encoder_count;
    int32_t count;
    int32_t last_count;
} qei_priv_t;

static Module g_qei_module;
static qei_priv_t g_qei_priv;

static void qei_update(Module *m)
{
    qei_priv_t *p = (qei_priv_t *)m->priv;
    if (!p) return;

    /* Read hardware counter from TIM2 (updates automatically on encoder edges) */
    volatile uint32_t *TIM2_CNT = (volatile uint32_t *)(TIM2_BASE + TIM_CNT);
    p->count = (int32_t)*TIM2_CNT;

    *p->ptr_encoder_count = (float)p->count;
    p->last_count = p->count;
}

void qei_init(void)
{
    /* Configure GPIO pins for TIM2 alternate function (AF1):
     * PA_0 = TIM2_CH1 (encoder phase A)
     * PA_1 = TIM2_CH2 (encoder phase B)
     */
    static GPIOPin pin_a, pin_b;
    gpio_init(&pin_a, "PA_0", GPIO_ALTERNATE, GPIO_NONE);
    gpio_init(&pin_b, "PA_1", GPIO_ALTERNATE, GPIO_NONE);

    /* Set AF1 on PA_0 and PA_1 (GPIOA_AFRL, bits for pins 0 and 1) */
    volatile uint32_t *GPIOA_AFRL = (volatile uint32_t *)(0x40020000 + 0x20);
    *GPIOA_AFRL = (*GPIOA_AFRL & ~(0xF << (0 * 4))) | (GPIO_AF_TIM2 << (0 * 4));
    *GPIOA_AFRL = (*GPIOA_AFRL & ~(0xF << (1 * 4))) | (GPIO_AF_TIM2 << (1 * 4));

    /* Enable TIM2 clock (APB1ENR bit 0) */
    volatile uint32_t *RCC_APB1ENR = (volatile uint32_t *)(0x40023800 + 0x40);
    *RCC_APB1ENR |= (1U << 0);

    /* Configure TIM2 in encoder mode:
     *  1. Disable counter during config
     *  2. CC1S=01, CC2S=01 (input capture, direct mapping on TI1/TI2)
     *  3. CC1E=1, CC2E=1 (enable capture)
     *  4. SMS=001 (encoder mode TI1FP1/TI2FP2)
     *  5. ARR=0xFFFFFFFF (full 32-bit range), PSC=0
     *  6. Enable counter
     */
    volatile uint32_t *TIM2_CR1   = (volatile uint32_t *)(TIM2_BASE + TIM_CR1);
    volatile uint32_t *TIM2_CCMR1 = (volatile uint32_t *)(TIM2_BASE + TIM_CCMR1);
    volatile uint32_t *TIM2_CCER  = (volatile uint32_t *)(TIM2_BASE + TIM_CCER);
    volatile uint32_t *TIM2_SMCR  = (volatile uint32_t *)(TIM2_BASE + TIM_SMCR);
    volatile uint32_t *TIM2_ARR   = (volatile uint32_t *)(TIM2_BASE + TIM_ARR);
    volatile uint32_t *TIM2_PSC   = (volatile uint32_t *)(TIM2_BASE + TIM_PSC);
    volatile uint32_t *TIM2_EGR   = (volatile uint32_t *)(TIM2_BASE + TIM_EGR);
    volatile uint32_t *TIM2_SR    = (volatile uint32_t *)(TIM2_BASE + TIM_SR);

    *TIM2_CR1 = 0; /* disable counter during config */

    *TIM2_CCMR1 = TIM_CCMR1_CC1S_INPUT | TIM_CCMR1_CC2S_INPUT;
    *TIM2_CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;

    *TIM2_SMCR = TIM_SMCR_SMS_ENCODER_TI12;
    *TIM2_ARR  = 0xFFFFFFFFU;
    *TIM2_PSC  = 0;

    *TIM2_EGR  = 1; /* reinit counter */
    *TIM2_SR   = 0; /* clear status flags */

    *TIM2_CR1 |= TIM_CR1_CEN; /* enable counter */

    /* Initialise module */
    g_qei_module.priv = &g_qei_priv;
    module_init(&g_qei_module, (int32_t)PRU_BASEFREQ, 0);
    g_qei_priv.ptr_encoder_count = &txData.processVariable[0];
    g_qei_priv.count = 0;
    g_qei_priv.last_count = 0;
    g_qei_module.update = qei_update;
    timer_register_module(TIMER_BASE, &g_qei_module, false);
    miniprintf("Creating QEI, hardware TIM2 encoder mode (PA_0/TIM2_CH1, PA_1/TIM2_CH2)\n");
}
