/*
 * estop.c — eStop / emergency-stop module
 * Port of Mbed eStop.h/cpp (FirmwareSource/Remora-OS5/eStop.h/cpp)
 *
 * Reads the eStop GPIO pin on the servo thread. When the eStop input is
 * asserted (active high with pull-none / external pull-down), the module
 * drives txData.header = PRU_ESTOP so LinuxCNC sees the emergency stop,
 * and sets g_pru_estop so the state machine can react.
 *
 * Pin: PA_2 (free — QEI index on PA_2 is not wired up in qei.c).
 * Direction: input, no pull (expects external pull-down; asserted = high).
 */

#include "estop.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile bool *ptr_estop;
    GPIOPin pin_obj;
    bool last_state;
} estop_priv_t;

static Module g_estop_module;
static estop_priv_t g_estop_priv;

static void estop_update(Module *m)
{
    estop_priv_t *p = (estop_priv_t *)m->priv;
    if (!p) return;

    bool state = gpio_get(&p->pin_obj);

    /* Rising edge: eStop just asserted */
    if (state && !p->last_state) {
        txData.header = PRU_ESTOP;
        *p->ptr_estop = true;
        miniprintf("eStop asserted\n");
    } else if (!state) {
        *p->ptr_estop = false;
    }

    p->last_state = state;
}

void estop_init(void)
{
    static const char *estop_pin_name = "PA_2";

    g_estop_module.priv = &g_estop_priv;
    module_init(&g_estop_module, (int32_t)PRU_SERVOFREQ, 0);
    g_estop_priv.ptr_estop = &g_pru_estop;
    g_estop_priv.last_state = false;
    gpio_init(&g_estop_priv.pin_obj, estop_pin_name, GPIO_INPUT, GPIO_PULLNONE);
    g_estop_module.update = estop_update;
    timer_register_module(TIMER_SERVO, &g_estop_module, false);
    miniprintf("Create eStop at pin %s\n", estop_pin_name);
}
