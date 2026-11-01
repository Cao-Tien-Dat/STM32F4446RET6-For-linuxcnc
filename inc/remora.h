#ifndef REMORA_H
#define REMORA_H

#include <stdint.h>
#include <stdbool.h>

#define SPI_BUFF_SIZE   64
#define JOINTS          5
#define VARIABLES       1

/* SPI command headers (same as Mbed firmware) */
#define PRU_DATA    0x64617461UL   /* "data" */
#define PRU_READ    0x72656164UL   /* "read" */
#define PRU_WRITE   0x77726974UL   /* "writ" */
#define PRU_ESTOP   0x65737470UL   /* "estp" */

/*
 * Packed SPI data structures — identical layout to Mbed remora.h
 * so LinuxCNC HAL component can communicate without changes.
 */
#pragma pack(push, 1)

typedef union {
    uint8_t  rxBuffer[SPI_BUFF_SIZE];
    struct {
        int32_t           header;
        volatile int32_t  jointFreqCmd[JOINTS];
        float             setPoint[VARIABLES];
        uint8_t           jointEnable;
        uint16_t          outputs;
        uint8_t           spare0;
    };
} rxData_t;

typedef union {
    uint8_t  txBuffer[SPI_BUFF_SIZE];
    struct {
        int32_t   header;
        int32_t   jointFeedback[JOINTS];
        float     processVariable[VARIABLES];
        uint16_t  inputs;
    };
} txData_t;

#pragma pack(pop)

extern volatile rxData_t rxData;
extern volatile txData_t txData;
extern volatile bool g_pru_reset;
extern volatile bool g_pru_estop;

#endif /* REMORA_H */
