/*
 * timer.c — Timer ISR driver for bare-metal scheduler
 * Replaces Mbed pruTimer / pruThread (TARGET_STM32F4/thread/timer.cpp)
 *
 * TIM9  → Base thread   (120 kHz, IRQ priority 2)
 * TIM10 → Servo thread  (1 kHz,   IRQ priority 3)
 * TIM11 → Comms thread  (57.6 kHz, IRQ priority 4)
 */

#include "timer.h"
#include "module.h"
#include "spi.h"

#include <stdint.h>

/* Timer base addresses */
#define TIM9_BASE  0x40014C00
#define TIM10_BASE 0x40015000
#define TIM11_BASE 0x40015400
#define TIM2_BASE  0x40000000

/* TIM register offsets */
#define TIM_CR1   0x00
#define TIM_CR2   0x04
#define TIM_PSC   0x28
#define TIM_ARR   0x2C
#define TIM_EGR   0x14
#define TIM_DIER  0x0C
#define TIM_SR    0x10

/* TIM_CR1 bits */
#define TIM_CR1_CEN   (1U << 0)
#define TIM_CR1_UDIS  (1U << 1)
#define TIM_CR1_URS   (1U << 2)

/* TIM_DIER bits */
#define TIM_DIER_UIE  (1U << 0)

/* TIM_SR bits */
#define TIM_SR_UIF    (1U << 0)

/* TIM_EGR bits */
#define TIM_EGR_UG    (1U << 0)

/* NVIC */
#define NVIC_ISER_BASE 0xE000E100
#define NVIC_IPR_BASE  0xE000E400
#define NVIC_ISER0     (*(volatile uint32_t *)(NVIC_ISER_BASE))
#define NVIC_IPR(n)    (*(volatile uint32_t *)(NVIC_IPR_BASE + (n) * 4))

/* APB2ENR / APB1ENR for timer clocks */
#define RCC_APB2ENR (*(volatile uint32_t *)(0x40023800 + 0x44))
#define RCC_APB1ENR (*(volatile uint32_t *)(0x40023800 + 0x40))

/* Module callback arrays — fixed size, no dynamic allocation */
#define MAX_MODULES_PER_THREAD 32

static Module *g_base_modules[MAX_MODULES_PER_THREAD]   = { NULL };
static Module *g_servo_modules[MAX_MODULES_PER_THREAD]  = { NULL };
static Module *g_comms_modules[MAX_MODULES_PER_THREAD]  = { NULL };
static int     g_base_count   = 0;
static int     g_servo_count  = 0;
static int     g_comms_count  = 0;

static Module *g_base_post[MAX_MODULES_PER_THREAD]   = { NULL };
static Module *g_servo_post[MAX_MODULES_PER_THREAD]  = { NULL };
static Module *g_comms_post[MAX_MODULES_PER_THREAD]  = { NULL };
static int     g_base_post_count   = 0;
static int     g_servo_post_count  = 0;
static int     g_comms_post_count  = 0;

static void timer_reg_write(uint32_t tim_base, uint32_t offset, uint32_t val)
{
    *(volatile uint32_t *)(tim_base + offset) = val;
}

static uint32_t timer_reg_read(uint32_t tim_base, uint32_t offset)
{
    return *(volatile uint32_t *)(tim_base + offset);
}

/* NVIC ISER/IPR register access — split across ISER0/ISER1 for IRQs >= 32 */
#define NVIC_ISER1     (*(volatile uint32_t *)(NVIC_ISER_BASE + 0x04))

static void timer_config_irq(uint32_t irq_number, uint32_t priority)
{
    /* Enable interrupt in the correct NVIC ISER register */
    if (irq_number < 32) {
        NVIC_ISER0 |= (1U << irq_number);
    } else {
        NVIC_ISER1 |= (1U << (irq_number - 32));
    }
    /* Write a single priority byte (upper 4 bits = priority, lower 4 bits = subpriority) */
    *(volatile uint8_t *)(NVIC_IPR_BASE + irq_number) = (uint8_t)((priority << 4) & 0xFF);
}

void timer_init(void)
{
    /* Timer clocks will be enabled per-timer in timer_start() */
}

void timer_start(uint32_t timer_id, uint32_t frequency_hz)
{
    uint32_t tim_base;
    uint32_t irq;
    uint32_t apb_clk;
    uint32_t psc = 4; /* TIM_PSC from Mbed timer.cpp */
    uint32_t arr;

    switch (timer_id) {
    case TIMER_BASE:
        tim_base = TIM9_BASE;
        irq = 24; /* TIM1_BRK_TIM9_IRQn */
        RCC_APB2ENR |= (1U << 19); /* TIM9EN  (APB2ENR bit 19) */
        apb_clk = 84000000U;       /* APB2 = 84 MHz */
        break;
    case TIMER_SERVO:
        tim_base = TIM10_BASE;
        irq = 25; /* TIM1_UP_TIM10_IRQn */
        RCC_APB2ENR |= (1U << 20); /* TIM10EN (APB2ENR bit 20) */
        apb_clk = 84000000U;       /* APB2 = 84 MHz */
        break;
    case TIMER_COMMS:
        tim_base = TIM11_BASE;
        irq = 26; /* TIM1_TRG_COM_TIM11_IRQn */
        RCC_APB2ENR |= (1U << 21); /* TIM11EN (APB2ENR bit 21) */
        apb_clk = 84000000U;       /* APB2 = 84 MHz */
        break;
    case TIMER_QEI:
        tim_base = TIM2_BASE;
        irq = 28; /* TIM2_IRQn */
        RCC_APB1ENR |= (1U << 0);  /* TIM2EN  (APB1ENR bit 0) */
        apb_clk = 42000000U;       /* APB1 = 42 MHz */
        break;
    default:
        return;
    }

    /* Disable timer during config */
    timer_reg_write(tim_base, TIM_CR1, 0);

    /* Prescaler */
    timer_reg_write(tim_base, TIM_PSC, psc - 1);

    /* Auto-reload: freq = clk / (psc) / (arr + 1) */
    arr = (apb_clk / psc / frequency_hz) - 1;
    timer_reg_write(tim_base, TIM_ARR, arr);

    /* Reinit counter */
    timer_reg_write(tim_base, TIM_EGR, TIM_EGR_UG);

    /* Enable update interrupt */
    timer_reg_write(tim_base, TIM_DIER, TIM_DIER_UIE);

    /* Clear pending flag (read-modify-write: status bits cleared by writing 0) */
    timer_reg_write(tim_base, TIM_SR, timer_reg_read(tim_base, TIM_SR) & ~TIM_SR_UIF);

    /* Enable NVIC */
    timer_config_irq(irq, timer_id == TIMER_BASE ? 2 :
                     timer_id == TIMER_SERVO ? 3 : 4);

    /* Enable timer */
    timer_reg_write(tim_base, TIM_CR1, TIM_CR1_CEN);
}

void timer_stop(uint32_t timer_id)
{
    uint32_t tim_base;
    switch (timer_id) {
    case TIMER_BASE:  tim_base = TIM9_BASE;  break;
    case TIMER_SERVO: tim_base = TIM10_BASE; break;
    case TIMER_COMMS: tim_base = TIM11_BASE; break;
    case TIMER_QEI:   tim_base = TIM2_BASE;  break;
    default: return;
    }
    timer_reg_write(tim_base, TIM_CR1, 0);
}

/* Module registration (called during setup) */
void timer_register_module(uint32_t timer_id, Module *mod, bool post)
{
    if (post) {
        switch (timer_id) {
        case TIMER_BASE:
            if (g_base_post_count < MAX_MODULES_PER_THREAD)
                g_base_post[g_base_post_count++] = mod;
            break;
        case TIMER_SERVO:
            if (g_servo_post_count < MAX_MODULES_PER_THREAD)
                g_servo_post[g_servo_post_count++] = mod;
            break;
        case TIMER_COMMS:
            if (g_comms_post_count < MAX_MODULES_PER_THREAD)
                g_comms_post[g_comms_post_count++] = mod;
            break;
        default: break;
        }
    } else {
        switch (timer_id) {
        case TIMER_BASE:
            if (g_base_count < MAX_MODULES_PER_THREAD)
                g_base_modules[g_base_count++] = mod;
            break;
        case TIMER_SERVO:
            if (g_servo_count < MAX_MODULES_PER_THREAD)
                g_servo_modules[g_servo_count++] = mod;
            break;
        case TIMER_COMMS:
            if (g_comms_count < MAX_MODULES_PER_THREAD)
                g_comms_modules[g_comms_count++] = mod;
            break;
        default: break;
        }
    }
}

/* Vector table aliases — map STM32 IRQ names to our ISR functions.
 * The startup vector table references the STM32 peripheral IRQ names;
 * our ISR implementations have different names, so we alias them here.
 */
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((alias("timer_base_isr")));
void TIM1_UP_TIM10_IRQHandler(void) __attribute__((alias("timer_servo_isr")));
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__((alias("timer_comms_isr")));
void TIM2_IRQHandler(void);

/* ISR implementations — run all modules for this thread */
void timer_base_isr(void)
{
    /* Clear UIF (read-modify-write: status bits cleared by writing 0) */
    timer_reg_write(TIM9_BASE, TIM_SR, timer_reg_read(TIM9_BASE, TIM_SR) & ~TIM_SR_UIF);

    for (int i = 0; i < g_base_count; i++)
        module_run(g_base_modules[i]);
    for (int i = 0; i < g_base_post_count; i++)
        module_run_post(g_base_post[i]);
}

void timer_servo_isr(void)
{
    timer_reg_write(TIM10_BASE, TIM_SR, timer_reg_read(TIM10_BASE, TIM_SR) & ~TIM_SR_UIF);

    for (int i = 0; i < g_servo_count; i++)
        module_run(g_servo_modules[i]);
    for (int i = 0; i < g_servo_post_count; i++)
        module_run_post(g_servo_post[i]);
}

void timer_comms_isr(void)
{
    timer_reg_write(TIM11_BASE, TIM_SR, timer_reg_read(TIM11_BASE, TIM_SR) & ~TIM_SR_UIF);

    for (int i = 0; i < g_comms_count; i++)
        module_run(g_comms_modules[i]);
    for (int i = 0; i < g_comms_post_count; i++)
        module_run_post(g_comms_post[i]);
}

/* TIM2 is used in polled encoder mode (qei.c reads TIM2->CNT directly).
 * The update interrupt is enabled by timer_start() for completeness, so
 * provide a handler that just clears any pending update flag and returns.
 */
void TIM2_IRQHandler(void)
{
    timer_reg_write(TIM2_BASE, TIM_SR, timer_reg_read(TIM2_BASE, TIM_SR) & ~TIM_SR_UIF);
}
