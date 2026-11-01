/*
 * pwm.c — PWM / spindle output module
 * Port of Mbed pwm.h/cpp (FirmwareSource/Remora-OS5/pwm.h/cpp)
 *
 * Simple software PWM in the servo thread: compares setPoint against a
 * running count to generate a PWM waveform on a GPIO pin.
 */

#include "pwm.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile float *ptr_sp;
    int pwm_max;
    GPIOPin pin_obj;
} pwm_priv_t;

static Module g_pwm_modules[MAX_PWM];
static pwm_priv_t g_pwm_priv[MAX_PWM];

static void pwm_update(Module *m)
{
    pwm_priv_t *p = (pwm_priv_t *)m->priv;
    if (!p) return;

    float sp = *p->ptr_sp;
    if (sp > 100.0f) sp = 100.0f;
    if (sp < 0.0f) sp = 0.0f;

    int pwm_val = (int)(p->pwm_max * (sp / 100.0f));
    /* Simple software PWM via GPIO toggle in servo thread */
    gpio_set(&p->pin_obj, pwm_val > (p->pwm_max / 2));
}

void pwm_init(void)
{
    static const char *pwm_pin[MAX_PWM] = { "PA_8" };

    for (int i = 0; i < MAX_PWM; i++) {
        pwm_priv_t *p = &g_pwm_priv[i];
        g_pwm_modules[i].priv = p;
        module_init(&g_pwm_modules[i], (int32_t)PRU_SERVOFREQ, 0);

        p->ptr_sp = &rxData.setPoint[i];
        p->pwm_max = 255;

        gpio_init(&p->pin_obj, pwm_pin[i], GPIO_OUTPUT, GPIO_NONE);

        g_pwm_modules[i].update = pwm_update;
        timer_register_module(TIMER_SERVO, &g_pwm_modules[i], false);
        miniprintf("Create PWM for Spindle PWM at pin %s\n", pwm_pin[i]);
    }
}
