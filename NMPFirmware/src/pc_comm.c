/*
 * pc_comm.c
 *
 *  Created on: Oct 6, 2021
 *      Author: austi
 */

#include <pc_comm.h>
#include <msp430.h>
#include <string.h>
#include <gpio_devices.h>
#include <aht10.h>
#include <stdio.h>

//Define Globals

volatile char uca0_rx_buffer[uca0_rx_buffer_len + 1];
volatile unsigned int uca0_rx_buffer_pos = 0;

//Initialize UART0

void init_pc_uart(void){

    // P1.1 = RXD, P1.2 = TXD
    P1SEL  |=  BIT1 + BIT2;
    P1SEL2 |=  BIT1 + BIT2;

    // For details on where the baud rate settings come from see
    // https://www.ti.com/lit/ug/slau144j/slau144j.pdf
    // Page 424 has a helpful table
    UCA0CTL1 |= UCSSEL_2 + UCSWRST;   // UART Clock = SMCLK = 1MHz
    UCA0BR0 = 104;         // Baud rate setting for 9600 baud at 1MHz
    UCA0BR1 = 0;           // Baud rate setting for 9600 baud at 1MHz
    UCA0MCTL = UCBRS_1;    // Modulation setting for 9600 baud at 1MHz
    UCA0CTL1 &= ~UCSWRST;  // Initialize UART

    // Enable receive interrupt
    IE2 |= UCA0RXIE;

    // Send start message
    send_string_to_pc(START_MSG);
    send_string_to_pc(PROMPT);
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void pc_comm_rx_isr(void){
    // If the buffer is full discard old data.
    // This ensures that a \n will not be ignored
    if(uca0_rx_buffer_pos == uca0_rx_buffer_len){
        uca0_rx_buffer_pos = 0;
    }

    // Echo character
    if(UCA0RXBUF != '\n'){
        UCA0TXBUF = UCA0RXBUF;
    }

    if(UCA0RXBUF == 127){
        // PuTTY uses 127 for backspace
        // Don't put backspace in buffer
        // Remove previous character from buffer
        uca0_rx_buffer_pos--;
    }

    else if(UCA0RXBUF != '\n' && UCA0RXBUF != '\r'){
        // Put in buffer if not \n or \r (line feed of carriage return)
        uca0_rx_buffer[uca0_rx_buffer_pos] = UCA0RXBUF;
        uca0_rx_buffer_pos++;
    }

    // If complete command (enter was pressed) handle the received data
    if(UCA0RXBUF == '\r' || (UCA0RXBUF == '\n' && uca0_rx_buffer_pos > 0)){
        // Exit LPM0 after interrupt exits
        __bic_SR_register_on_exit(CPUOFF);
    }

    // Clear RX interrupt flag
    IFG2 &= ~UCA0RXIFG;
}

void send_byte_to_pc(char byte){
    while (!(IFG2 & UCA0TXIFG)); // Wait for current TX to finish
    UCA0TXBUF = byte;
}

void send_string_to_pc(const char *message){
    unsigned int i;
    for(i = 0; i < strlen(message); ++i){
        while (!(IFG2&UCA0TXIFG)); // Wait for current TX to finish
        UCA0TXBUF = message[i];
    }
}

void pc_comm_process(void){
    // Put in low power mode 0 (CPU off) with interrupts enabled
    // Waiting for something to happen that needs some action to be performed
    __bis_SR_register(LPM0_bits + GIE);

    // The ISR will exit LPM0 when a complete command has been received
    // In other words, these next lines only run after a
    // complete command has been received

    // Also echo '\n' after receiving \r b/c PuTTY is weird...
    // Pressing enter only sends \r, but putty expects \r\n
    // for a newline to display properly
    // The \r will have already been sent (because it was echoed when received)
    send_byte_to_pc('\n');

    // Append null terminator (so string operations work as intended)
    uca0_rx_buffer[uca0_rx_buffer_pos] = '\0';

    // Handle complete command
    if(strcmp((const char*)uca0_rx_buffer, "help") == 0){
        // Response to help command
        send_string_to_pc(HELP_MSG);
    }
    else if(strcmp((const char*)uca0_rx_buffer, "on") == 0){
        // Enable sensors
        enable_sensors();

        // Wait 500ms for temp sensor to power on and be ready for I2C communication
        // 1 cycle = 1/MCLK = 1/1MHz = 1 micro second
        // 500ms = 500000 cycles
        __delay_cycles(500000);

        IE2 &= ~UCA0RXIE; // Disable UART RX interrupt while I2C TX occurs
        aht10_init();
        IE2 |= UCA0RXIE;  // Re-enable

        send_string_to_pc(SENSOR_ON_MSG);
    }
    else if(strcmp((const char*)uca0_rx_buffer, "off") == 0){
        // Disable sensors
        disable_sensors();
        send_string_to_pc(SENSOR_OFF_MSG);
    }
    else if(strcmp((const char*)uca0_rx_buffer, "temp") == 0){
        // Send temperature data
        char temp_str[10];
        float temp = aht10_read_temp();
        sprintf(temp_str, "%d.%d\r\n", (int)temp, (int)((temp - (int)temp) * 100));
        send_string_to_pc(temp_str);
    }
    else{
        // Unknown command
        send_string_to_pc(UNKNOWN_MSG);
    }

    // Show prompt again
    send_string_to_pc(PROMPT);
    // Clear rx buffer
    uca0_rx_buffer_pos = 0;
}




