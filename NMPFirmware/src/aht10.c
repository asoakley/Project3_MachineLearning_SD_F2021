/*
 * aht10.c
 *
 *  Created on: Oct 13, 2021
 *      Author: austi
 */

#include <msp430.h>
#include <aht10.h>
#include <stdint.h>
#include <stdbool.h>

// Definition of buffers (declared in header)
volatile uint8_t *ucb0_tx_buffer = 0;
volatile uint8_t ucb0_tx_count = 0;
volatile uint8_t *ucb0_rx_buffer = 0;
volatile uint8_t ucb0_rx_count = 0;

// Constants used in I2C communication with the AHT10
#define AHT10_ADDRESS             0x38 // I2C address of sensor module

#define AHT10_INIT_CMD            0xE1 // Init command
#define AHT10_CMD_CALIBRATE       0x08 // Load factory calibration
#define AHT10_CMD_NORMAL          0xA8 // Normal cycle mode
#define AHT10_CMD_START_MEASURE   0xAC // Start measurement
#define AHT10_MEASUREMENT_MODE    0x33 // Probably DAC resolution
#define AHT10_CMD_RESET           0xBA // Reset sensor
#define AHT10_CMD_NOP             0x00 // no operation

// Pre-defined data
unsigned char AHT10_CALIBRATE[3] = {AHT10_INIT_CMD, AHT10_CMD_CALIBRATE, AHT10_CMD_NOP};
unsigned char AHT10_NORMAL[3] = {AHT10_CMD_NORMAL, AHT10_CMD_NOP, AHT10_CMD_NOP};
unsigned char AHT10_MEASURE[3] = {AHT10_CMD_START_MEASURE, AHT10_MEASUREMENT_MODE, AHT10_CMD_NOP};

////////////////////////////////////////////////////////////////////////////////
// Functions for I2C communication
////////////////////////////////////////////////////////////////////////////////
// Notice that these functions were not declared in the header. It is allowed to
// create functions that are not declared in the header. Such functions will not
// be accessible from other source files, but these functions are only used here
// to simplify the code in aht10_init and aht10_read_temp, so they are not
// needed outside this source file.
void i2c_b0_init(){
    // Configure P1.6 and P1.7 for I2C
    P1SEL  |= BIT6 + BIT7;
    P1SEL2 |= BIT6 + BIT7;

    // Put UCB0 in reset so it is able to be configured
    UCB0CTL1 = UCSWRST;

    // I2C Master mode synchronous
    UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC;

    // SMCLK = 1MHz / 10 = 100kHz
    // a standard I2C clock rate that should work with the AHT10
    UCB0BR0 = 10;
    UCB0BR1 = 0;

    // Take out of reset and use SMCLK as source clock
    UCB0CTL1 = UCSSEL_2;
}

bool i2c_b0_is_error(void){
    // If NACK
    if (UCB0STAT & UCNACKIFG) {
        // Send stop condition
        UCB0CTL1 |= UCTXSTP;
        // Clear flag
        UCB0STAT &= ~UCNACKIFG;
        // Error occurred, return true
        return true;
    }
    return false;
}

void i2c_b0_tx(uint8_t slave_address, uint8_t *data, uint8_t count){
    // Wait until bus not busy
    while(UCB0STAT & UCBUSY);

    // Select target device
    UCB0I2CSA = slave_address;

    // Send start condition
    UCB0CTL1 |= UCTR | UCTXSTT;

    // Wait for start condition to be sent
    // And ready to transmit
    while ((UCB0CTL1 & UCTXSTT) && !(IFG2 & UCB0TXIFG));

    while(count > 0){
        UCB0TXBUF = *data; // Transmit next byte
        while(!(IFG2 & UCB0TXIFG)){
            if(i2c_b0_is_error()){
                count = 0; // Done transmitting. Error occurred
                break;
            }
        }
        // Move to next byte
        data++;
        count--;
    }

    // Send stop condition (done writing)
    UCB0CTL1 |= UCTXSTP;
    while (UCB0CTL1 & UCTXSTP);
}


void i2c_b0_rx(uint8_t slave_address, uint8_t *buffer, uint8_t count){

    while(UCB0STAT & UCBUSY); // Wait until bus not busy

    // Select target device
    UCB0I2CSA = slave_address;

    // Send start condition (and take out of transmit mode)
    UCB0CTL1 &= ~UCTR;
    UCB0CTL1 |= UCTXSTT;
    while (UCB0CTL1 & UCTXSTT);

    // While still bytes to receive and no error
    while(!i2c_b0_is_error() && (count > 0)){
        // Send stop condition when only one byte left to receive
        if(count == 1){
            UCB0CTL1 |= UCTXSTP;
            while (UCB0CTL1 & UCTXSTP);
        }

        while (!(IFG2 & UCB0RXIFG)); // Wait for data
        *buffer = UCB0RXBUF;
        buffer++;
        count--;
    }

}



////////////////////////////////////////////////////////////////////////////////
// Higher level functions for using the aht10
////////////////////////////////////////////////////////////////////////////////

void aht10_init(void){
    i2c_b0_init();
    i2c_b0_tx(AHT10_ADDRESS, AHT10_CALIBRATE, 3);
    i2c_b0_tx(AHT10_ADDRESS, AHT10_NORMAL, 3);
}

float aht10_read_temp(void){
    uint8_t raw_data[6];

    i2c_b0_tx(AHT10_ADDRESS, AHT10_MEASURE, 3);
    i2c_b0_rx(AHT10_ADDRESS, raw_data, 6);

    uint32_t temperature = ((uint32_t) (raw_data[3] & 0x0F) << 16) | ((uint16_t)raw_data[4] << 8) | raw_data[5];
    return (float) temperature * 0.000191 - 50;

}



