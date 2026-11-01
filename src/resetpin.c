/*
 * resetpin.c — Reset / estop pin module
 * Port of Mbed resetPin.h/cpp (FirmwareSource/Remora-OS5/resetPin.h/cpp)
 *
 * Reads the reset pin state and exposes it via g_pru_reset (Servo thread).
 */

#include "resetpin.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile bool *ptr_reset;
    GPIOPin pin_obj;
} resetpin_priv_t;

static Module g_reset_pin_module;
static resetpin_priv_t g_reset_pin_priv;

static void resetpin_update(Module *m)
{
    resetpin_priv_t *p = (resetpin_priv_t *)m->priv;
    if (!p) return;
    *p->ptr_reset = gpio_get(&p->pin_obj);
}

void resetpin_init(void)
{
    static const char *reset_pin_name = "PC_3";

    g_reset_pin_module.priv = &g_reset_pin_priv;
    module_init(&g_reset_pin_module, (int32_t)PRU_SERVOFREQ, 0);
    g_reset_pin_priv.ptr_reset = &g_pru_reset;
    gpio_init(&g_reset_pin_priv.pin_obj, reset_pin_name, GPIO_INPUT, GPIO_PULLNONE);
    g_reset_pin_module.update = resetpin_update;
    timer_register_module(TIMER_SERVO, &g_reset_pin_module, false);
    miniprintf("Create Reset Pin at pin %s\n", reset_pin_name);
}
