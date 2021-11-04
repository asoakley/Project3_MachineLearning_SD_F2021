#include <msp430.h>
#include <gpio_devices.h>
#include <pc_comm.h>
#include <clocks.h>


// This function runs when the program starts
int main(void){
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    init_clocks();
    init_gpio();
    init_pc_uart();

    // Main loop
    while(1){
        // Will enter LPM0 while waiting for UART activity
        pc_comm_process();
    }
}
