/*
 * aht10.h
 *
 *  Created on: Oct 13, 2021
 *      Author: austi
 */

#ifndef INCLUDE_AHT10_H_
#define INCLUDE_AHT10_H_

// uint8_t is defined in this header
#include <stdint.h>

// Read and write buffers for USCI_B0 I2C communication
// These are declared here, but defined in the source file (extern)
// Recall that in c, arrays are pointers to the first item.
// As such keeping track of the length separately is necessary
extern volatile uint8_t *ucb0_tx_buffer;
extern volatile uint8_t ucb0_tx_count;
extern volatile uint8_t *ucb0_rx_buffer;
extern volatile uint8_t ucb0_rx_count;

void aht10_init(void);

float aht10_read_temp(void);


#endif /* INCLUDE_AHT10_H_ */
