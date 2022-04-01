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

//=======================
// Commands and Registers
//=======================

// Slave address
#define BMI160_ADDRESS          0x69

// Write to this register to change gyroscope operation mode
#define PMU_TRIGGER             0x6C
#define DISABLE_GYRO            0x00

// This register triggers operations like softreset, NVM programming, etc
#define CMD_REG                 0x7E

#define ACCEL_NORMAL            0x11

// This register is used to configure the accelerometer filter bandwidth and data rate
#define ACC_CONF                0x40

#define ACC_DR                 0x2C    // Disable undersampling, default bandwidth, 1600 Hz output data rate

// This register is used to configure the g-range
#define ACC_RANGE               0x41

#define RANGE_2G                0x03

#define RANGE_4G                0x05

#define RANGE_8G                0x08

#define RANGE_16G               0x0A

// Read this register for errors
#define ERR_REG                 0x02

// Acceleration Data Registers
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


// Accelerometer buffers

#define ACCEL_BUFFER_SIZE           128

// Function Declarations

void Init_I2CB1(uint32_t prescale);
void I2CB1_Send1(uint8_t slaveAddr, uint8_t data);
uint8_t I2CB1_Recv1(uint8_t slaveAddr);
void I2CB1_SendMultiple(uint8_t slaveAddr, uint8_t *data, uint8_t count);
void I2CB1_RecvMultiple(uint8_t slaveAddr, uint8_t regAddr, int8_t *buffer, uint8_t count);
bool I2CB1_Error(void);

void Init_IMU(void);

void IMU_Read_Accel(void);

void IMU_Read_Error(void);

void IMU_Clear_Buffers(void);



#endif /* INCLUDE_IMU_H_ */
