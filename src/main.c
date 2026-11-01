/*
 * main.c — Remora Flexi-HAL bare-metal firmware main entry
 * Port of Mbed main.cpp (FirmwareSource/Remora-OS5/main.cpp)
 *
 * State machine: ST_SETUP → ST_START → ST_IDLE → ST_RUNNING → ST_RESET → ST_WDRESET
 * Main loop services watchdog and manages SPI communication state.
 * Module logic (stepgen, digitalpin, PWM, QEI, resetpin) lives in separate files.
 */

#include "remora.h"
#include "configuration.h"
#include "module.h"
#include "timer.h"
#include "spi.h"
#include "gpio.h"
#include "miniprintf.h"

#include "stepgen.h"
#include "digitalpin.h"
#include "pwm.h"
#include "qei.h"
#include "resetpin.h"
#include "estop.h"
#include "usart.h"
#include "config.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Forward declarations */
static void setup(void);
static void load_modules(void);

/* SPI data — defined here (extern in remora.h) */
volatile rxData_t rxData;
volatile txData_t txData;

/* Global state machine */
typedef enum {
    ST_SETUP    = 0,
    ST_START    = 1,
    ST_IDLE     = 2,
    ST_RUNNING  = 3,
    ST_STOP     = 4,
    ST_RESET    = 5,
    ST_WDRESET  = 6
} state_t;

volatile bool g_pru_reset   = false;
volatile bool g_pru_estop   = false;
static bool          g_config_error = false;
static bool          g_threads_running = false;
static uint8_t       g_reset_cnt   = 0;

/* -------------------------------------------------------------------------- */
/* Module loading                                                             */
/* -------------------------------------------------------------------------- */

static void load_modules(void)
{
    if (g_config_error) return;

    miniprintf("\n5. Loading Flexi modules\n");

    stepgen_init();
    digitalpin_init();
    qei_init();
    resetpin_init();
    pwm_init();
    estop_init();
}

/* -------------------------------------------------------------------------- */
/* Setup                                                                      */
/* -------------------------------------------------------------------------- */

static void setup(void)
{
    miniprintf("\n2. Setting up DMA and threads\n");

    spi_comms_init();
    spi_comms_start();
}

/* -------------------------------------------------------------------------- */
/* Independent Watchdog (IWDG) — 2000 ms timeout                              */
/* LSI ~32 kHz, prescaler /64 -> 2 ms per tick; reload 999 -> ~2000 ms.       */
/* Once started the watchdog cannot be stopped (option-byte dependent).       */
/* -------------------------------------------------------------------------- */

#define IWDG_BASE      0x40003000
#define IWDG_KR        (*(volatile uint32_t *)(IWDG_BASE + 0x00))
#define IWDG_PR        (*(volatile uint32_t *)(IWDG_BASE + 0x04))
#define IWDG_RLR       (*(volatile uint32_t *)(IWDG_BASE + 0x08))
#define IWDG_SR        (*(volatile uint32_t *)(IWDG_BASE + 0x0C))

#define IWDG_KR_KEY_ENABLE   0xCCCC  /* Start watchdog */
#define IWDG_KR_KEY_RELOAD   0xAAAA  /* Reload counter (kick) */
#define IWDG_KR_KEY_WRITE_EN 0x5555  /* Enable write access to PR/RLR */

#define IWDG_PR_DIV64 4              /* /64 prescaler */
#define IWDG_RLR_2S   999            /* 999 * 2 ms ~= 2000 ms */

static void iwdg_init(void)
{
    /* Enable write access to prescaler/reload registers */
    IWDG_KR = IWDG_KR_KEY_WRITE_EN;
    /* Wait for prescaler value update to complete */
    while (IWDG_SR & 0x01) { }       /* PVU */
    IWDG_PR = IWDG_PR_DIV64;
    while (IWDG_SR & 0x02) { }       /* RVU */
    IWDG_RLR = IWDG_RLR_2S;
    while (IWDG_SR & 0x02) { }       /* RVU */
    IWDG_KR = IWDG_KR_KEY_RELOAD;    /* Reload counter */
    IWDG_KR = IWDG_KR_KEY_ENABLE;    /* Start watchdog */
}

static void iwdg_kick(void)
{
    IWDG_KR = IWDG_KR_KEY_RELOAD;
}

/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    state_t current_state = ST_SETUP;
    state_t prev_state = ST_WDRESET;

    spi_set_status(false);
    spi_set_error(false);

    /* Initialize USART early so the startup miniprintf output works */
    usart_init();

    miniprintf("\nRemora PRU - Programmable Realtime Unit\n");
    miniprintf("\nLoading - Expatria Flexi-HAL\n");

    /* Configure and start the independent watchdog (2000 ms) */
    iwdg_init();

    while (1) {
        /* Kick the watchdog each main-loop pass */
        iwdg_kick();

        switch (current_state) {
        case ST_SETUP:
            if (current_state != prev_state) {
                miniprintf("\n## Entering SETUP state\n");
            }
            prev_state = current_state;

            setup();
            readJsonConfig();
            deserialiseJSON();
            configThreads();
            timer_init();
            /* timer_start for Base, Servo, Comms — done after modules registered */
            load_modules();

            /* Start the timer threads after modules are registered */
            timer_start(TIMER_BASE,   (uint32_t)PRU_BASEFREQ);   /* 120 kHz */
            timer_start(TIMER_SERVO,  (uint32_t)PRU_SERVOFREQ);  /* 1 kHz */
            timer_start(TIMER_COMMS,  (uint32_t)PRU_COMMSFREQ);  /* 57.6 kHz */

            current_state = ST_START;
            break;

        case ST_START:
            if (current_state != prev_state) {
                miniprintf("\n## Entering START state\n");
            }
            prev_state = current_state;

            if (!g_threads_running) {
                miniprintf("\nStarting the BASE thread\n");
                miniprintf("\nStarting the SERVO thread\n");
                g_threads_running = true;

                /* Wait for threads to read IO before testing for PRUreset */
                for (volatile int i = 0; i < 1680000; i++) __asm__("nop");
                iwdg_kick();
            }

            if (g_pru_reset) {
                current_state = ST_START;
            } else {
                current_state = ST_IDLE;
            }
            break;

        case ST_IDLE:
            if (current_state != prev_state) {
                miniprintf("\n## Entering IDLE state\n");
            }
            prev_state = current_state;

            if (spi_get_error()) {
                miniprintf("Communication data error\n");
                spi_set_error(false);
                spi_comms_error_recovery();
            }

            if (spi_get_status()) {
                miniprintf("\n## Entering RUNNING state\n");
                current_state = ST_RUNNING;
            }

            if (g_pru_reset) {
                miniprintf("\n## Entering PRU_RESET state\n");
                current_state = ST_WDRESET;
            }
            break;

        case ST_RUNNING:
            if (current_state != prev_state) {
                miniprintf("\n## Entering RUNNING state\n");
            }
            prev_state = current_state;

            if (spi_get_error()) {
                miniprintf("Communication data error\n");
                spi_set_error(false);
                spi_comms_error_recovery();
            }

            if (spi_get_status()) {
                g_reset_cnt = 0;
                spi_set_status(false);
            } else {
                g_reset_cnt++;
            }

            if (g_reset_cnt >= SPI_ERR_MAX) {
                miniprintf("   Communication data error limit reached, resetting\n");
                g_reset_cnt = 0;
                current_state = ST_RESET;
            }

            if (g_pru_reset) {
                current_state = ST_WDRESET;
            }
            break;

        case ST_STOP:
            if (current_state != prev_state) {
                miniprintf("\n## Entering STOP state\n");
            }
            prev_state = current_state;
            current_state = ST_IDLE;
            break;

        case ST_RESET:
            if (current_state != prev_state) {
                miniprintf("\n## Entering RESET state\n");
            }
            prev_state = current_state;

            miniprintf("   Resetting rxBuffer\n");
            {
                int n = sizeof(rxData.rxBuffer);
                volatile uint8_t *buf = rxData.rxBuffer;
                while (n-- > 0) buf[n] = 0;
            }

            current_state = ST_IDLE;
            break;

        case ST_WDRESET:
            miniprintf("\n## Entering WDRESET state\n");
            while (1) { /* force watchdog reset */ }
            break;
        }

        /* Sleep until next interrupt (timer ISR / EXTI), then kick watchdog */
        __asm__("wfi");
    }

    return 0;
}

/* _write is provided by miniprintf.c — retarget to USART1 there if needed */
