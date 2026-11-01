/*
 * digitalpin.c — Digital input/output modules
 * Port of Mbed digitalPin.h/cpp (FirmwareSource/Remora-OS5/digitalPin.h/cpp)
 *
 * Digital outputs:  write rxData.outputs bits to GPIO pins (Servo thread)
 * Digital inputs:   read GPIO pins into txData.inputs bits (Servo thread)
 */

#include "digitalpin.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile uint16_t *ptr_data;
    int bit_number;
    int mask;
    int mode;        /* 0=input, 1=output */
    bool invert;
    GPIOPin pin_obj;
} digitalpin_priv_t;

static Module g_do_modules[MAX_DO];
static digitalpin_priv_t g_do_priv[MAX_DO];
static Module g_di_modules[MAX_DI];
static digitalpin_priv_t g_di_priv[MAX_DI];

static void digitalpin_update(Module *m)
{
    digitalpin_priv_t *p = (digitalpin_priv_t *)m->priv;
    if (!p) return;

    if (p->mode == 0) {
        /* Input */
        bool pin_state = gpio_get(&p->pin_obj);
        if (p->invert) pin_state = !pin_state;
        if (pin_state)
            *p->ptr_data |= p->mask;
        else
            *p->ptr_data &= ~p->mask;
    } else {
        /* Output */
        bool pin_state = (*p->ptr_data & p->mask) ? true : false;
        if (p->invert) pin_state = !pin_state;
        gpio_set(&p->pin_obj, pin_state);
    }
}

void digitalpin_init(void)
{
    volatile uint16_t *ptr_inputs  = &txData.inputs;
    volatile uint16_t *ptr_outputs = &rxData.outputs;

    static const char *do_name[MAX_DO] = {
        "AUX2", "AUX3", "MIST", "AUX1", "SPINDLE_EN",
        "AUX0", "COOLANT", "STEP_EN", "STEP_EN_Z", "SPINDLE_DIR"
    };
    static const char *do_pin[MAX_DO] = {
        "PA_4", "PA_6", "PA_7", "PB_0", "PB_2",
        "PB_13", "PC_9", "PA_13", "PA_14", "PB_1"
    };
    static bool do_invert[MAX_DO] = {
        false, false, false, false, false,
        false, false, false, false, true
    };
    static int do_bit[MAX_DO] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

    static const char *di_name[MAX_DI] = {
        "X_LIMIT", "A_LIMIT", "PROBE", "Y_LIMIT", "KPSTR",
        "DOOR", "HALT", "FD_HLD", "CYC_START", "Z_LIMIT", "B_LIMIT"
    };
    static const char *di_pin[MAX_DI] = {
        "PA_5", "PB_6", "PB_7", "PB_9", "PB_10",
        "PC_4", "PB_12", "PC_8", "PC_11", "PC_13", "PC_14"
    };
    static bool di_invert[MAX_DI] = {
        true, true, true, true, true,
        true, true, true, true, true, true
    };
    static int di_bit[MAX_DI] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

    /* Digital outputs (Servo thread) */
    int do_count = sizeof(do_name) / sizeof(do_name[0]);
    for (int i = 0; i < do_count && i < MAX_DO; i++) {
        digitalpin_priv_t *p = &g_do_priv[i];
        g_do_modules[i].priv = p;
        module_init(&g_do_modules[i], (int32_t)PRU_SERVOFREQ, 0);

        p->ptr_data = ptr_outputs;
        p->bit_number = do_bit[i];
        p->mask = 1 << do_bit[i];
        p->mode = 1;
        p->invert = do_invert[i];

        gpio_init(&p->pin_obj, do_pin[i], GPIO_OUTPUT, GPIO_NONE);

        g_do_modules[i].update = digitalpin_update;
        timer_register_module(TIMER_SERVO, &g_do_modules[i], false);
        miniprintf("Create digital output for %s\n", do_name[i]);
    }

    /* Digital inputs (Servo thread) */
    int di_count = sizeof(di_name) / sizeof(di_name[0]);
    for (int i = 0; i < di_count && i < MAX_DI; i++) {
        digitalpin_priv_t *p = &g_di_priv[i];
        g_di_modules[i].priv = p;
        module_init(&g_di_modules[i], (int32_t)PRU_SERVOFREQ, 0);

        p->ptr_data = ptr_inputs;
        p->bit_number = di_bit[i];
        p->mask = 1 << di_bit[i];
        p->mode = 0;
        p->invert = di_invert[i];

        gpio_init(&p->pin_obj, di_pin[i], GPIO_INPUT, GPIO_PULLNONE);

        g_di_modules[i].update = digitalpin_update;
        timer_register_module(TIMER_SERVO, &g_di_modules[i], false);
        miniprintf("Create digital input for %s\n", di_name[i]);
    }
}
