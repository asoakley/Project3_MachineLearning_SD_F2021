/*
 * IMU.h
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"
#include <stdbool.h>

#ifndef INCLUDE_IMU_H_
#define INCLUDE_IMU_H_

//==========
// Commands
//==========

#define BMI160_ADDRESS          0x69

// Write to this register to change gyroscope operation mode
#define PMU_TRIGGER             0x6C

// Read this register for errors
#define ERR_REG                 0x02

#define ACC_X_LSB               0x12
#define ACC_X_MSB               0x13

#define ACC_Y_LSB               0x14
#define ACC_Y_MSB               0x15

#define ACC_Z_LSB               0x16
#define ACC_Z_MSB               0x17

//======================
// Baud Rate Prescale
//======================

#define IMU_BAUD                120



//======================
// Data buffers for UART
//======================

extern uint8_t *ucb1_tx_buffer;
extern uint8_t ucb1_tx_count;
extern uint8_t *ucb1_rx_buffer;
extern uint8_t ucb1_rx_count;



// Accelerometer values struct

typedef struct{
    uint16_t x_accel;
    uint16_t y_accel;
    uint16_t z_accel;
} ACCELXYZ_TYPE;

// Function Declarations

void Init_I2CB1(uint32_t prescale);
void I2CB1_Send1(uint8_t slaveAddr, uint8_t data);
uint8_t I2C_Recv1(uint8_t slaveAddr);
void I2CB1_SendMultiple(uint8_t slaveAddr, uint8_t *data, uint8_t count);
void I2C_RecvMultiple(uint8_t slaveAddr, uint8_t *buffer, uint8_t count);
bool I2CB1_Error(void);

void Init_IMU(void);

ACCELXYZ_TYPE IMU_Read_Accel(void);

void IMU_Read_Error(void);



#endif /* INCLUDE_IMU_H_ */
