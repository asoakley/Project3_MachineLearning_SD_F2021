/*
 * IMU.h
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"

#ifndef INCLUDE_IMU_H_
#define INCLUDE_IMU_H_

extern uint8_t *ucb1_tx_buffer;
extern uint8_t ucb1_tx_count;
extern uint8_t *ucb1_rx_buffer;
extern uint8_t ucb1_rx_count;

void Init_I2CB1(uint32_t prescale);
void I2CB1_Send1(uint8_t slaveAddr, uint8_t data);
uint8_t I2C_Recv1(int8_t slaveAddr);



void Init_IMU(void);

void IMU_read_data(void);




#endif /* INCLUDE_IMU_H_ */
