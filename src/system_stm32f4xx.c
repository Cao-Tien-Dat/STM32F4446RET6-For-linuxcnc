/*
 * system_stm32f4xx.c — CMSIS system clock initialization for STM32F446RE
 *
 * Matches custom_targets.json clock configuration:
 *   HSE: 25 MHz XTAL (USE_PLL_HSE_XTAL | USE_PLL_HSI)
 *   USB: OTG_FS
 *   LSE: not available
 *
 * Target system clock: 168 MHz (max for STM32F446RE at 2.7-3.6V)
 *   PLL: HSE/25 * 168 / 2 = 168 MHz
 *   AHB: 168 MHz
 *   APB1: 42 MHz (max 45 MHz)
 *   APB2: 84 MHz (max 90 MHz)
 */

#include <stdint.h>

/* RCC registers */
#define RCC_BASE          0x40023800
#define RCC_CR            (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR       (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR          (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_CIR           (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_AHB1RSTR      (*(volatile uint32_t *)(RCC_BASE + 0x10))
#define RCC_AHB2RSTR      (*(volatile uint32_t *)(RCC_BASE + 0x14))
#define RCC_AHB3RSTR      (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB1RSTR      (*(volatile uint32_t *)(RCC_BASE + 0x20))
#define RCC_APB2RSTR      (*(volatile uint32_t *)(RCC_BASE + 0x24))
#define RCC_AHB1ENR       (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_AHB2ENR       (*(volatile uint32_t *)(RCC_BASE + 0x34))
#define RCC_AHB3ENR       (*(volatile uint32_t *)(RCC_BASE + 0x38))
#define RCC_APB1ENR       (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR       (*(volatile uint32_t *)(RCC_BASE + 0x44))
#define RCC_DCKCFGR       (*(volatile uint32_t *)(RCC_BASE + 0x8C))
#define RCC_DCKCFGR2      (*(volatile uint32_t *)(RCC_BASE + 0x90))

/* RCC_CR bits */
#define RCC_CR_HSION          (1U << 0)
#define RCC_CR_HSIRDY         (1U << 1)
#define RCC_CR_HSEON          (1U << 16)
#define RCC_CR_HSERDY         (1U << 17)
#define RCC_CR_HSEBYP         (1U << 18)
#define RCC_CR_CSSON          (1U << 19)
#define RCC_CR_PLLON          (1U << 24)
#define RCC_CR_PLLRDY         (1U << 25)

/* RCC_PLLCFGR bits */
#define RCC_PLLCFGR_PLLM_POS  0
#define RCC_PLLCFGR_PLLN_POS  6
#define RCC_PLLCFGR_PLLP_POS  16
#define RCC_PLLCFGR_PLLQ_POS  24
#define RCC_PLLCFGR_PLLSRC    (1U << 22)

/* RCC_CFGR bits */
#define RCC_CFGR_SW_POS       0
#define RCC_CFGR_SW_HSI       0x00000000
#define RCC_CFGR_SW_HSE       0x00000001
#define RCC_CFGR_SW_PLL       0x00000002
#define RCC_CFGR_SWS_POS      2
#define RCC_CFGR_HPRE_POS     4
#define RCC_CFGR_PPRE1_POS    10
#define RCC_CFGR_PPRE2_POS    13
#define RCC_CFGR_MCO1_POS     21
#define RCC_CFGR_I2SSRC_POS   23
#define RCC_CFGR_MCO1PRE_POS  26
#define RCC_CFGR_MCO2PRE_POS  30

/* FLASH registers */
#define FLASH_BASE          0x40023C00
#define FLASH_ACR           (*(volatile uint32_t *)(FLASH_BASE + 0x00))

#define FLASH_ACR_LATENCY_POS 0
#define FLASH_ACR_PRFTEN      (1U << 8)

/* SystemCoreClock */
uint32_t SystemCoreClock = 168000000U;

/*
 * SystemInit — called from startup before main()
 * Configure PLL for 168 MHz system clock from 25 MHz HSE.
 */
void SystemInit(void)
{
    /* 1. Enable HSI (reset default) — already on after reset */
    while (!(RCC_CR & RCC_CR_HSIRDY)) { /* wait */ }

    /* 2. Enable HSE (25 MHz external crystal) */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY)) { /* wait for HSE ready */ }

    /* 3. Set Flash latency: 5 wait states for 168 MHz (2.7-3.6V) */
    FLASH_ACR = (FLASH_ACR & ~0x07) | 0x05;
    FLASH_ACR |= FLASH_ACR_PRFTEN;

    /* 4. Configure PLL:
     *    PLLM = 25   (HSE / 25 = 1 MHz)
     *    PLLN = 168  (1 MHz * 168 = 168 MHz VCO)
     *    PLLP = 2    (168 / 2 = 84 MHz? Actually PLLP=2 means divide by 2 -> 84... wait)
     *    Correction: PLLP division factor: 0=2, 1=4, 2=6, 3=8
     *    For 168 MHz SYSCLK: PLLN=168, PLLP=0 (divide by 2) -> VCO=336, SYSCLK=168
     *    Wait: VCO = fPLLIN * PLLN = 1 * 168 = 168 MHz (too low for VCO min 100 MHz)
     *    Let's use: PLLM=25 (1 MHz input), PLLN=336 (336 MHz VCO), PLLP=0 (divide by 2) -> 168 MHz SYSCLK
     *    PLLQ=7 (48 MHz for USB: 336/7=48)
     */
    RCC_PLLCFGR = (25 << RCC_PLLCFGR_PLLM_POS) |   /* PLLM = 25 */
                  (336 << RCC_PLLCFGR_PLLN_POS) |  /* PLLN = 336 */
                  (0 << RCC_PLLCFGR_PLLP_POS) |    /* PLLP = 2 */
                  (7 << RCC_PLLCFGR_PLLQ_POS) |    /* PLLQ = 7 */
                  RCC_PLLCFGR_PLLSRC;              /* Source = HSE */

    /* 5. Enable PLL */
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY)) { /* wait for PLL ready */ }

    /* 6. Configure bus dividers:
     *    AHB prescaler = 1 (168 MHz)
     *    APB1 prescaler = 4 (42 MHz)
     *    APB2 prescaler = 2 (84 MHz)
     */
    RCC_CFGR = (0x0 << RCC_CFGR_HPRE_POS) |   /* AHB = SYSCLK / 1 */
               (0x5 << RCC_CFGR_PPRE1_POS) |  /* APB1 = HCLK / 4 */
               (0x4 << RCC_CFGR_PPRE2_POS);   /* APB2 = HCLK / 2 */

    /* 7. Switch system clock to PLL (preserve HPRE/PPRE1/PPRE2 prescalers) */
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while (((RCC_CFGR >> RCC_CFGR_SWS_POS) & 0x03) != 0x02) { /* wait */ }

    /* 8. Update SystemCoreClock */
    SystemCoreClock = 168000000U;
}
