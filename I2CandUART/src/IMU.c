/*
 * IMU.c
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"
#include <IMU.h>
#include <gpio.h>

void Init_I2CB1(uint32_t prescale){
    EUSCI_B1->CTLW0 |= UCSWRST;     // Put in reset


    EUSCI_B1->CTLW0 = 0x0FC1;
    // bit 15       UCA10 = 0; Own address is 7-bit
    // bit 14       UCSLA10 = 0; slave address is 7-bit
    // bit 13       UCMM = 0; single master environment
    // bit 11       UCMST = 1; master mode
    // bits 10-9    UCMODEx = 3; I2C mode
    // bit 8        UCSYNC = 1; synchronous mode
    // bit 7-6      UCSSELx = 3; eUSCI clock SMCLK = 3MHz
    // bit 0        UCSWRST = 1;

    EUSCI_B1->CTLW1 = 0;
    // bits 7-6     UCCLTO = 0; disable timeout clock
    // bit 5        UCSTPNACK = 0; send negative ack before stop in receive
    // bit 4        UCSWACK = 0; slave address ack controlled by hardware
    // bits 3-2     UCASTPx = 0; no automatic stop condition after UBC0TBCNT
    // bits 1-0     UCGLITx = 0; deglitch time of 50 ns

    EUSCI_B1->BRW = prescale; // set to 120 for 100 KHz and set to 30 for 400 KHz

    P6SEL0 |= (UCB1SDA | UCB1SCL);     // P6.4, P6.5 SDA and SCL function
    P6SEL1 &= ~(UCB1SDA | UCB1SCL);     // P6.4, P6.5 SDA and SCL function

    EUSCI_B1->CTLW0 &= ~UCSWRST;        // take out of reset
    EUSCI_B1->IE = 0x0000;      // disable all interrupts ???????????

}

void I2CB1_Send1(uint8_t slaveAddr, uint8_t data){
    while(EUSCI_B1->STATW & UCBBUSY){};
    EUSCI_B1->I2CSA = slaveAddr;    // Set slave address
    EUSCI_B1->CTLW0 &= ~UCTXSTP;    // No stop
    EUSCI_B1->CTLW0 |= (UCTR | UCTXSTT); // Master transmit, start condition
    while((EUSCI_B1->IFG & UCTXIFG0) == 0){}; // wait for Transmit Interrupt Flag (raised when buffer is empty in master mode)
    EUSCI_B1->TXBUF = data; // Put byte in the buffer
    while((EUSCI_B1->IFG & UCTXIFG0) == 0){}; // wait for the Transmit Interrupt Flag again
    EUSCI_B1->CTLW0 |= UCTXSTP; // Generate Stop condition
    EUSCI_B1->IFG &= ~UCTXIFG0; // clear UCTXIFG0

}

uint8_t I2C_Recv1(int8_t slaveAddr){
    int8_t data;
    while(EUSCI_B1->STATW & UCBBUSY){}; // wait for I2C to be ready
    EUSCI_B1->CTLW0 |= UCSWRST; // Put module in reset to modify TBCNT
    EUSCI_B1->TBCNT = 1; // generate STOP after 1 byte
    EUSCI_B1->CTLW0 &= ~UCSWRST; // Take out of reset
    EUSCI_B1->I2CSA = slaveAddr; // set slave address
    EUSCI_B1->CTLW0 &= ~UCTR;    // Put Module in receive mode
    EUSCI_B1->CTLW0 |= (UCTXSTP | UCTXSTT);
    while((EUSCI_B1->IFG & UCRXIFG0) == 0){}; // If slave address is wrong, this hangs up!!
    data = EUSCI_B1->RXBUF; // get data from the slave
    return data;

}

