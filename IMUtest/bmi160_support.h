/*
*
****************************************************************************
* Copyright (C) 2016 Bosch Sensortec GmbH
*
* File : bmi160_support.h
*
* Date : 2016/03/15
*
* Revision : 1.0.7 $
*
* Usage: Sensor Driver support file for BMI160 sensor
*
****************************************************************************
*
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

/*! \file bmi160_support.h
    \brief BMI160 Sensor Driver Support Header File */
/* user defined code to be added here ... */

#ifndef BMI160_SUPPORT_H_
#define BMI160_SUPPORT_H_

#ifdef __cplusplus
extern "C" {
#endif



/*!
 * @brief struct used for assign the value for
 *	gyro sleep configuration
 */
struct gyro_sleep_setting {
	u8 sleep_trigger;/**< gyro sleep trigger configuration*/
	u8 wakeup_trigger;/**< gyro wakeup trigger configuration*/
	u8 sleep_state;/**< gyro sleep state configuration*/
	u8 wakeup_int;/**< gyro wakeup interrupt configuration*/
};
/********************************/
/**\name POWE MODES DEFINITION */
/*******************************/
#define ACCEL_MODE_NORMAL	(0x11)
#define GYRO_MODE_NORMAL	(0x15)
#define	ACCEL_LOWPOWER		(0X12)
#define MAG_SUSPEND_MODE	(1)
#define BMI160_MODE_SWITCHING_DELAY		(30)
/********************************/
/**\name RETURN TYPE */
/*******************************/
/* return type of communication routine*/
#define BMI160_RETURN_FUNCTION_TYPE s8
/********************************/
/**\name RUNNING MODE DEFINITIONS */
/*******************************/
#define	STANDARD_UI_9DOF_FIFO			(0)
#define	STANDARD_UI_IMU_FIFO			(1)
#define	STANDARD_UI_IMU					(2)
#define	STANDARD_UI_ADVANCEPOWERSAVE	(3)
#define	ACCEL_PEDOMETER					(4)
#define APPLICATION_HEAD_TRACKING		(5)
#define APPLICATION_NAVIGATION			(6)
#define APPLICATION_REMOTE_CONTROL		(7)
#define APPLICATION_INDOOR_NAVIGATION	(8)
/********************************/
/**\name MAG INTERFACE */
/*******************************/
#define	C_BMI160_BYTE_COUNT	   (2)
#define BMI160_SLEEP_STATE     (0x00)
#define BMI160_WAKEUP_INTR     (0x00)
#define BMI160_SLEEP_TRIGGER   (0x04)
#define BMI160_WAKEUP_TRIGGER  (0x02)
/*fifo wm set to 512 bytes*/
#define BMI160_ENABLE_FIFO_WM  (0x80)
#define	BMI160_MAG_INTERFACE_OFF_PRIMARY_ON		(0x00)
#define	BMI160_MAG_INTERFACE_ON_PRIMARY_ON		(0x02)

/********************************/
/**\name I2C INTERFACE */
/*******************************/
#define	I2C_BUFFER_LEN 8

/********************************/
/**\name GUICOMPOSER INTERFACE */
/*******************************/
#define MAX_STR_LENGTH  271
#define UART_BAUDRATE   9600
/*!
 *	@brief This function used for initialize the sensor
 *	@param i2cHndl : I2C handle used for communication with sensor
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_initialize_sensor(I2C_Handle i2cHndl);
/*!
 *	@brief This Function used to read the sensor data using
 *	different running mode
 *	@param v_running_mode_u8 : The value of running mode
 *      Description                |  value
 * --------------------------------|----------
 *  STANDARD_UI_9DOF_FIFO          |   0
 *	STANDARD_UI_IMU_FIFO           |   1
 *	STANDARD_UI_IMU                |   2
 *	STANDARD_UI_ADVANCEPOWERSAVE   |   3
 *	ACCEL_PEDOMETER                |   4
 *	APPLICATION_HEAD_TRACKING      |   5
 *	APPLICATION_NAVIGATION         |   6
 *	APPLICATION_REMOTE_CONTROL     |   7
 *	APPLICATION_INDOOR_NAVIGATION  |   8
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(u8 v_running_mode_u8);
/*!
 *	@brief This function used for interrupt configuration
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_interrupt_configuration(void);


#ifdef __cplusplus
}
#endif

#endif /* BMI160_SUPPORT_H_ */
