/*
 * gpio_devices.h
 *
 *  Created on: Oct 6, 2021
 *      Author: austi
 */

#ifndef INCLUDE_GPIO_DEVICES_H_
#define INCLUDE_GPIO_DEVICES_H_

//Function Declarations

__interrupt void blink_power_led(void);
void init_gpio(void);
void enable_sensors();
void disable_sensors();


#endif /* INCLUDE_GPIO_DEVICES_H_ */
