/*
 * PC_comm.h
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"

#ifndef INCLUDE_PC_COMM_H_
#define INCLUDE_PC_COMM_H_

//======================
// Data buffers for UART
//======================

extern uint8_t *ucb1_tx_buffer;
extern uint8_t ucb1_tx_count;
extern uint8_t *ucb1_rx_buffer;
extern uint8_t ucb1_rx_count;

//======================
// Function Declarations
//======================

void Init_PC_UART(void);
void Send_String_To_PC(const char *message);
void Print_Accel(void);

#endif /* INCLUDE_PC_COMM_H_ */
