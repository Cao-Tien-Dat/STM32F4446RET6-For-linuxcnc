#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "module.h"

/* Timer IDs matching Mbed thread mapping */
#define TIMER_BASE   9   /* TIM9  - Base thread  (120 kHz) */
#define TIMER_SERVO  10  /* TIM10 - Servo thread (1 kHz) */
#define TIMER_COMMS  11  /* TIM11 - Comms thread (57.6 kHz) */
#define TIMER_QEI     2  /* TIM2  - QEI encoder   */

void timer_init(void);
void timer_start(uint32_t timer_id, uint32_t frequency_hz);
void timer_stop(uint32_t timer_id);

/* Register a module with a timer thread (called during setup) */
void timer_register_module(uint32_t timer_id, Module *mod, bool post);

/* Called from timer IRQ handlers */
void timer_base_isr(void);
void timer_servo_isr(void);
void timer_comms_isr(void);

#endif /* TIMER_H */
