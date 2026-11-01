/*
 * gpio.c — GPIO driver for STM32F446RE bare-metal firmware
 * Replaces Mbed Pin class (TARGET_STM32F446RE/drivers/pin/pin.cpp)
 */

#include "gpio.h"
#include <stddef.h>

/* GPIO base addresses */
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define GPIOC_BASE 0x40020800
#define GPIOD_BASE 0x40020C00
#define GPIOE_BASE 0x40021000
#define GPIOF_BASE 0x40021400
#define GPIOG_BASE 0x40021800
#define GPIOH_BASE 0x40021C00

/* RCC AHB1ENR offsets */
#define RCC_AHB1ENR_OFFSET 0x30
#define RCC_BASE 0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET))

static void *gpio_base_from_index(uint8_t portIndex)
{
    switch (portIndex) {
        case 0: return (void *)GPIOA_BASE;
        case 1: return (void *)GPIOB_BASE;
        case 2: return (void *)GPIOC_BASE;
        case 3: return (void *)GPIOD_BASE;
        case 4: return (void *)GPIOE_BASE;
        case 5: return (void *)GPIOF_BASE;
        case 6: return (void *)GPIOG_BASE;
        case 7: return (void *)GPIOH_BASE;
        default: return NULL;
    }
}

static void gpio_enable_clock(uint8_t portIndex)
{
    switch (portIndex) {
        case 0: RCC_AHB1ENR |= (1U << 0); break; /* GPIOA */
        case 1: RCC_AHB1ENR |= (1U << 1); break; /* GPIOB */
        case 2: RCC_AHB1ENR |= (1U << 2); break; /* GPIOC */
        case 3: RCC_AHB1ENR |= (1U << 3); break; /* GPIOD */
        case 4: RCC_AHB1ENR |= (1U << 4); break; /* GPIOE */
        case 5: RCC_AHB1ENR |= (1U << 5); break; /* GPIOF */
        case 6: RCC_AHB1ENR |= (1U << 6); break; /* GPIOG */
        case 7: RCC_AHB1ENR |= (1U << 7); break; /* GPIOH */
        default: break;
    }
}

bool gpio_init(GPIOPin *pin, const char *name, int dir, int modifier)
{
    if (!pin || !name) return false;

    /* Parse "PA_4" -> portIndex, pinNumber */
    if (name[0] < 'A' || name[0] > 'H') return false;
    if (name[1] < 'A' || name[1] > 'Z') return false;
    if (name[2] != '_') return false;

    pin->portIndex = (uint8_t)(name[0] - 'A');
    pin->pinNumber = (uint8_t)(name[3] - '0');
    if (name[4] >= '0' && name[4] <= '9') {
        pin->pinNumber = (uint8_t)((pin->pinNumber * 10) + (name[4] - '0'));
    }
    if (pin->pinNumber > 15) return false;

    pin->pinMask = (uint16_t)(1U << pin->pinNumber);
    pin->dir = (uint8_t)dir;
    pin->modifier = (uint8_t)modifier;
    pin->GPIOx = gpio_base_from_index(pin->portIndex);
    if (!pin->GPIOx) return false;

    gpio_enable_clock(pin->portIndex);

    /* Configure GPIO mode */
    uint32_t mode;
    if (dir == GPIO_INPUT) {
        mode = 0x0; /* Input */
    } else if (dir == GPIO_ALTERNATE) {
        mode = 0x2; /* Alternate function */
    } else if (modifier == GPIO_OPENDRAIN) {
        mode = 0x1; /* General purpose output open-drain */
    } else {
        mode = 0x1; /* General purpose output push-pull */
    }

    /* GPIOx_MODER: 2 bits per pin */
    volatile uint32_t *MODER = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x00);
    *MODER = (*MODER & ~(0x3 << (pin->pinNumber * 2))) | (mode << (pin->pinNumber * 2));

    /* GPIOx_OTYPE: 1 bit per pin (open-drain) */
    volatile uint32_t *OTYPER = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x04);
    if (modifier == GPIO_OPENDRAIN) {
        *OTYPER |= (1U << pin->pinNumber);
    } else {
        *OTYPER &= ~(1U << pin->pinNumber);
    }

    /* GPIOx_OSPEEDR: 2 bits per pin (high speed) */
    volatile uint32_t *OSPEEDR = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x08);
    *OSPEEDR = (*OSPEEDR & ~(0x3 << (pin->pinNumber * 2))) | (0x2 << (pin->pinNumber * 2));

    /* GPIOx_PUPDR: 2 bits per pin */
    volatile uint32_t *PUPDR = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x0C);
    uint32_t pupd;
    switch (modifier) {
        case GPIO_PULLUP:    pupd = 0x1; break;
        case GPIO_PULLDOWN:  pupd = 0x2; break;
        default:             pupd = 0x0; break;
    }
    *PUPDR = (*PUPDR & ~(0x3 << (pin->pinNumber * 2))) | (pupd << (pin->pinNumber * 2));

    return true;
}

bool gpio_get(const GPIOPin *pin)
{
    if (!pin || !pin->GPIOx) return false;

    volatile uint32_t *IDR = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x10);
    return (*IDR & pin->pinMask) != 0;
}

void gpio_set(GPIOPin *pin, bool value)
{
    if (!pin || !pin->GPIOx) return;

    volatile uint32_t *BSRR = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x18);
    if (value) {
        *BSRR = pin->pinMask;               /* Set bit */
    } else {
        *BSRR = (uint32_t)pin->pinMask << 16; /* Reset bit */
    }
}

void gpio_toggle(GPIOPin *pin)
{
    if (!pin || !pin->GPIOx) return;

    volatile uint32_t *ODR = (volatile uint32_t *)((uint32_t)pin->GPIOx + 0x14);
    *ODR ^= pin->pinMask;
}
