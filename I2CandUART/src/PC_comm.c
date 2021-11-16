/*
 * PC_Comm.c
 *
 *  Created on: Oct 26, 2021
 *      Author: austi
 */

#include "msp.h"
#include <PC_comm.h>
#include <gpio.h>
#include <IMU.h>
#include <string.h>
#include <stdio.h>

//=============
// Globals
//=============

extern int16_t x_accel[ACCEL_BUFFER_SIZE];

extern int16_t y_accel[ACCEL_BUFFER_SIZE];

extern int16_t z_accel[ACCEL_BUFFER_SIZE];

extern uint16_t b_index;

//==========================
// Initialize EUSCI_A0 UART
//==========================

void Init_PC_UART(void){
    EUSCI_A0->CTLW0 |= UCSWRST; // Put module in reset for configuration
    EUSCI_A0->CTLW0 |= UCSSEL_2; // SMCLK (12MHz)
    EUSCI_A0->MCTLW = 0;              // disable oversampling
    EUSCI_A0->CTLW0 &= ~(UCSPB | UCPEN);      // 1 stop bit, parity disabled
    EUSCI_A0->BRW = 625; // Divider value = 12MHZ / 19200 baud
    P1SEL0 |= (UCA0TXD | UCA0RXD);     // Configure P1.2 and P1.3 for UART Function
    P1SEL1 &= ~(UCA0TXD | UCA0RXD);     // Configure P1.2 and P1.3 for UART Function
    EUSCI_A0->CTLW0 &= ~UCSWRST;        // Take out of reset
    EUSCI_A0->IFG |= UCRXIE;            // Enable receive interrupt

    NVIC_SetPriority(EUSCIA0_IRQn, 2);    // set interrupt priority, lower number is higher priority
    NVIC_EnableIRQ(EUSCIA0_IRQn);         // enable interrupt

    Send_String_To_PC("Hello World!\r\n");
}

void Send_String_To_PC(const char *message){
    unsigned int i;
    for(i = 0; i < strlen(message); ++i){
        while (!(EUSCI_A0->IFG & UCTXIFG)); // Wait for current TX to finish
        EUSCI_A0->TXBUF = message[i];
    }
}

void Print_Accel(void){
    char temp_str[40];
    IMU_Read_Accel();
    sprintf(temp_str, "X = %d  ,  Y = %d  ,  Z = %d\r\n", x_accel[b_index], y_accel[b_index], z_accel[b_index]);
    Send_String_To_PC(temp_str);
}
