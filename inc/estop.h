#ifndef ESTOP_H
#define ESTOP_H

#include "module.h"
#include "gpio.h"

/* Initialize eStop module (Servo thread).
 * Reads the eStop GPIO pin and asserts txData.header = PRU_ESTOP when active. */
void estop_init(void);

#endif /* ESTOP_H */
