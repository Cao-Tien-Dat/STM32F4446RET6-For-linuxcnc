/*
 * stepgen.c — Step generator module (DDS-based)
 * Port of Mbed stepgen.h/cpp (FirmwareSource/Remora-OS5/stepgen.h/cpp)
 *
 * Implements a DDS (Direct Digital Synthesis) step generator:
 *   frequency_scale = (1 << STEPBIT) / threadFreq
 *   dds_add_value   = freq_cmd * frequency_scale
 *   A step is generated when bit STEPBIT of the accumulator toggles.
 */

#include "stepgen.h"
#include "remora.h"
#include "configuration.h"
#include "timer.h"
#include "miniprintf.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int joint_number;
    int mask;
    bool is_enabled;
    bool is_forward;
    bool last_dir;
    bool is_stepping;
    int32_t dds_accumulator;
    int32_t dds_add_value;
    int step_bit;
    float frequency_scale;
    volatile int32_t *ptr_freq_cmd;
    volatile int32_t *ptr_feedback;
    volatile uint8_t *ptr_joint_enable;
    GPIOPin step_pin_obj;
    GPIOPin dir_pin_obj;
} stepgen_priv_t;

static Module g_stepgen_modules[JOINTS];
static stepgen_priv_t g_stepgen_priv[JOINTS];

static void stepgen_update(Module *m)
{
    stepgen_priv_t *p = (stepgen_priv_t *)m->priv;
    if (!p) return;

    p->is_enabled = ((*p->ptr_joint_enable & p->mask) != 0);

    if (p->is_enabled) {
        /* Multiply in float to preserve fractional scaling, then truncate */
        p->dds_add_value = (int32_t)((float)(*p->ptr_freq_cmd) * p->frequency_scale);
        int32_t step_now = p->dds_accumulator;
        p->dds_accumulator += p->dds_add_value;
        step_now ^= p->dds_accumulator;
        step_now &= (1L << p->step_bit);

        if (p->dds_add_value > 0)
            p->is_forward = true;
        else
            p->is_forward = false;

        if (p->last_dir != p->is_forward) {
            p->last_dir = p->is_forward;
            gpio_set(&p->dir_pin_obj, p->is_forward);
        } else if (step_now) {
            gpio_set(&p->step_pin_obj, true);
            *p->ptr_feedback = p->dds_accumulator >> p->step_bit;
        }
    }
}

static void stepgen_update_post(Module *m)
{
    stepgen_priv_t *p = (stepgen_priv_t *)m->priv;
    if (!p) return;
    gpio_set(&p->step_pin_obj, false);
}

void stepgen_init(void)
{
    volatile uint8_t *ptr_joint_enable = &rxData.jointEnable;

    static const char *step_pin[JOINTS]   = { "PA_3", "PC_1", "PB_8", "PD_2", "PB_14" };
    static const char *dir_pin[JOINTS]    = { "PC_2", "PC_0", "PC_15", "PC_12", "PB_15" };

    for (int i = 0; i < JOINTS; i++) {
        stepgen_priv_t *p = &g_stepgen_priv[i];
        g_stepgen_modules[i].priv = p;
        module_init(&g_stepgen_modules[i], (int32_t)PRU_BASEFREQ, 0);

        p->joint_number = i;
        p->mask = 1 << i;
        p->step_bit = STEPBIT;
        p->frequency_scale = (float)(1 << STEPBIT) / (float)PRU_BASEFREQ;
        p->ptr_freq_cmd = &rxData.jointFreqCmd[i];
        p->ptr_feedback = &txData.jointFeedback[i];
        p->ptr_joint_enable = ptr_joint_enable;
        p->is_enabled = false;
        p->is_forward = false;
        p->last_dir = !p->is_forward;
        p->dds_accumulator = 0;
        p->is_stepping = false;

        gpio_init(&p->step_pin_obj, step_pin[i], GPIO_OUTPUT, GPIO_NONE);
        gpio_init(&p->dir_pin_obj, dir_pin[i], GPIO_OUTPUT, GPIO_NONE);

        g_stepgen_modules[i].update = stepgen_update;
        g_stepgen_modules[i].updatePost = stepgen_update_post;

        timer_register_module(TIMER_BASE, &g_stepgen_modules[i], false);
        timer_register_module(TIMER_BASE, &g_stepgen_modules[i], true);
        miniprintf("Create step generator for Joint %d\n", i);
    }
}
