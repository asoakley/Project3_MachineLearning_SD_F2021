/*
 * IMU.c
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"
#include <IMU.h>
#include <gpio.h>

void Init_I2CB1(void){
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

    EUSCI_B1->BRW =

    P6SEL0 |= (UCB1SDA | UCB1SCL);     // P6.4, P6.5 SDA and SCL function
    P6SEL1 &= ~(UCB1SDA | UCB1SCL);     // P6.4, P6.5 SDA and SCL function

    EUSCI_B1->CTLW0 &= ~UCSWRST;        // take out of reset
    EUSCI_B1->IE = 0x0000;      // disable all interrupts ???????????

}
