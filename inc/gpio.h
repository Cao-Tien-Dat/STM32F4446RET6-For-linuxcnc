#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* Pin directions */
#define GPIO_INPUT    0
#define GPIO_OUTPUT   1
#define GPIO_ALTERNATE 2

/* Pin modifiers */
#define GPIO_NONE      0b000
#define GPIO_OPENDRAIN 0b001
#define GPIO_PULLUP    0b010
#define GPIO_PULLDOWN  0b011
#define GPIO_PULLNONE  0b100

typedef struct {
    uint8_t  portIndex;   /* 0=A, 1=B, 2=C, ... */
    uint8_t  pinNumber;   /* 0-15 */
    uint16_t pinMask;
    uint8_t  dir;
    uint8_t  modifier;
    void    *GPIOx;       /* STM32 GPIO_TypeDef* */
} GPIOPin;

/* Initialize a pin from "PA_4" style name */
bool gpio_init(GPIOPin *pin, const char *name, int dir, int modifier);

/* Pin accessors */
bool gpio_get(const GPIOPin *pin);
void gpio_set(GPIOPin *pin, bool value);
void gpio_toggle(GPIOPin *pin);

#endif /* GPIO_H */
