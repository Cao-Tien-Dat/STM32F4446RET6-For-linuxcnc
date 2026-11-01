#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "remora.h"

/* SPI1 slave for LinuxCNC communication */
void spi_comms_init(void);
void spi_comms_start(void);

/* Re-enable SPI and DMA after an error (called from main loop error path) */
void spi_comms_error_recovery(void);

/* Status flags (set by ISR, read by main loop) */
bool spi_get_status(void);
void spi_set_status(bool status);
bool spi_get_error(void);
void spi_set_error(bool error);

/* Called from EXTI15_10_IRQHandler when NSS rising edge detected */
void spi_process_packet(void);

#endif /* SPI_H */
