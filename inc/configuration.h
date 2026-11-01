#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdint.h>

/* Thread frequencies (Hz) */
#define PRU_BASEFREQ        120000UL   /* Base thread ISR: stepgen, encoder */
#define PRU_SERVOFREQ       1000UL     /* Servo thread ISR: digital I/O, PWM */
#define OVERSAMPLE              3
#define PRU_COMMSFREQ    (19200UL * OVERSAMPLE)   /* 57600 Hz - comms watchdog */

/* Stepgen module configuration */
#define JOINTS              5          /* Number of joints (stepgen modules) */

/* Module array sizes */
#define MAX_DO          10
#define MAX_DI          11
#define MAX_PWM         1
#define STEPBIT           22             /* bit location in DDS accumulator */
#define STEP_MASK           (1L << STEPBIT)

/* SPI error handling */
#define LOOP_TIME           0.1f        /* main loop period (s) */
#define SPI_ERR_MAX         5           /* reset after 5 consecutive SPI errors */
                                          /* = 0.5s at LOOP_TIME=0.1s */

/* JSON configuration */
#define JSON_BUFF_SIZE      10000

/* Watchdog timeout (ms) */
#define WATCHDOG_TIMEOUT_MS 2000

/* UART stdio */
#define STDIO_BAUD          115200

#endif /* CONFIGURATION_H */
