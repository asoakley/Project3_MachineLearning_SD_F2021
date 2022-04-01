/*
****************************************************************************
* Copyright (C) 2016-2020 Bosch Sensortec GmbH
*
* bmi160_support.c
* Date: 2016/03/15
* Revision: 1.0.7 $
*
* Usage: Sensor Driver support file for BMI160 sensor
*
****************************************************************************
* License:
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

#ifndef DebugP_ASSERT_ENABLED
#define DebugP_ASSERT_ENABLED 0
#endif
#ifndef DebugP_LOG_ENABLED
#define DebugP_LOG_ENABLED 0
#endif

#include <ti/drivers/dpl/DebugP.h>

/* POSIX Header files */
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>

/* TI-Drivers Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

#include <ti/display/Display.h>

/* Module Header */
#include <ti/sail/bmi160/bmi160.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* example includes */
#include "bmi160_support.h"

/************** I2C buffer length ******/


#define DISPLAYTASKSTACKSIZE   512
#define BMI_TASK_STACK_SIZE    1024

static I2C_Handle     i2cHandle    = NULL;
sem_t bmi160Sem , displaySem;
/* Mapping the structure*/
struct bmi160_t s_bmi160;
/* Read the sensor data of accel, gyro and mag*/
struct bmi160_gyro_t gyroxyz = {0,0,0};
struct bmi160_accel_t accelxyz = {0,0,0};
struct bmi160_mag_xyz_s32_t magxyz = {0,0,0};
struct gyro_sleep_setting gyr_setting;
struct bmi160_fifo_data_header_t header_data;

//Calibration off-sets
int8_t accel_off_x;
int8_t accel_off_y;
int8_t accel_off_z;
int16_t gyro_off_x;
int16_t gyro_off_y;
int16_t gyro_off_z;

extern Display_Handle display;

pthread_t cbTask;
pthread_t dispTask;
/*----------------------------------------------------------------------------*/
/*  The following functions are used for reading and writing of
 *	sensor data using I2C or SPI communication
 *----------------------------------------------------------------------------*/
/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *        will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *        which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
static BMI160_RETURN_FUNCTION_TYPE BMI160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
static BMI160_RETURN_FUNCTION_TYPE BMI160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *        will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *        which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
static BMI160_RETURN_FUNCTION_TYPE BMI160_I2C_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt);

/*
 * \Brief: SPI/I2C init routine
*/
static BMI160_RETURN_FUNCTION_TYPE I2C_routine(void);

/********************End of I2C/SPI function declarations*********************/
/*	Brief : The delay routine
 *	\param : delay in ms
*/
static void BMI160_delay_msek(u32 msek);


/*--------------------------------------------------------------------------*/
/*	The following function is used to map the I2C bus read, write, delay and
 *	device address with global structure bmi160
 *-------------------------------------------------------------------------*/
static BMI160_RETURN_FUNCTION_TYPE I2C_routine(void)
{
/*--------------------------------------------------------------------------*/
/*	By using bmi160 the following structure parameter can be accessed
 *	Bus write function pointer: bmi160_WR_FUNC_PTR
 *	Bus read function pointer: bmi160_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	s_bmi160.bus_write = BMI160_I2C_bus_write;
	s_bmi160.bus_read = BMI160_I2C_bus_read;
	s_bmi160.burst_read = BMI160_I2C_burst_read;	
	s_bmi160.delay_msec = BMI160_delay_msek;
	s_bmi160.dev_addr = BMI160_I2C_ADDR2;

	return BMI160_INIT_VALUE;
}

/*
 *  ======== bmi160Callback ========
 *  When an ALERTing condition is met on the bmi160 hardware, the INT0 or INT1
 *  pin is asserted generating an interrupt. This callback function serves as
 *  an ISR for a single bmi160 sensor.
 */
static void bmi160Callback(uint_least8_t index)
{
    sem_post(&bmi160Sem);
}

static void* displayTask(void *arg0)
{

	while (1) {
		sem_wait(&displaySem);
		Display_print3(display, 0, 0, "accelo: x = %d, y = %d,z = %d\n",
						accelxyz.x, accelxyz.y, accelxyz.z);
		Display_print3(display, 0, 0, "gyro  : x = %d, y = %d,z = %d\n",
						gyroxyz.x, gyroxyz.y, gyroxyz.z);
		Display_print3(display, 0, 0, "magno : x = %d, y = %d,z = %d\n",
						magxyz.x, magxyz.y, magxyz.z);
	}
}

/*
 *  ======== bmiInterruptHandlerTask ========
 *  This task is unblocked when the bmi interrupt is triggered.
 */
void* bmiInterruptHandlerTask(void *arg0)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
	unsigned int i;
	while (1) {

		/* Pend on semaphore, bmi160Sem */
		if (0 == sem_wait(&bmi160Sem)) {
				com_rslt += bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
						&header_data);
				for(i=0;i<=header_data.accel_frame_count;i++)
				{
					if(i == 0)
					{
						accelxyz.x =  header_data.accel_fifo[i].x;
						accelxyz.y =  header_data.accel_fifo[i].y;
						accelxyz.z =  header_data.accel_fifo[i].z;
					}
					else
					{
						accelxyz.x = (accelxyz.x +
								header_data.accel_fifo[i].x)/2;
						accelxyz.y = (accelxyz.y +
								header_data.accel_fifo[i].y)/2;
						accelxyz.z = (accelxyz.z +
								header_data.accel_fifo[i].z)/2;
					}
				}
				for(i=0;i<=header_data.gyro_frame_count;i++)
				{
					if(i == 0)
					{
						gyroxyz.x =  header_data.gyro_fifo[i].x;
						gyroxyz.y =  header_data.gyro_fifo[i].y;
						gyroxyz.z =  header_data.gyro_fifo[i].z;
					}
					else
					{
						gyroxyz.x = (gyroxyz.x + header_data.gyro_fifo[i].x)/2;
						gyroxyz.y = (gyroxyz.y + header_data.gyro_fifo[i].y)/2;
						gyroxyz.z = (gyroxyz.z + header_data.gyro_fifo[i].z)/2;
					}
				}
				for(i=0;i<=header_data.mag_frame_count;i++)
				{
					if(i == 0)
					{
						magxyz.x =  header_data.mag_fifo[i].x;
						magxyz.y =  header_data.mag_fifo[i].y;
						magxyz.z =  header_data.mag_fifo[i].z;
					}
					else
					{
						magxyz.x = (magxyz.x + header_data.mag_fifo[i].x)/2;
						magxyz.y = (magxyz.y + header_data.mag_fifo[i].y)/2;
						magxyz.z = (magxyz.z + header_data.mag_fifo[i].z)/2;
					}
				}
				sem_post(&displaySem);
			}
		}
}

/*!
 *	@brief This function used for initialize the sensor.
 *
 *  This function used for initialize the sensor in 9DoF fifo mode.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_initialize_sensor(I2C_Handle i2cHndl)
{
	pthread_attr_t       pAttrs;
	struct sched_param   priParam;    
	int retc;
	
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
	
	i2cHandle = i2cHndl;
	/*	Based on the user need configure I2C or SPI interface.
	*	It is sample code to explain how to use the bmi160 API*/
	com_rslt = I2C_routine();
	
	/* The semaphore is used by the bmi160 interrupt handler task
	* the semaphore is normally posted by bmi160 interrupt(watermark)
	* The interrupt handler task waits on this semaphore.
	*/
	if(0 != sem_init(&bmi160Sem,0,0))
	{
		/* sem_init() failed */
		Display_print0(display, 0, 0, "bmi160sem Semaphore creation failed");
		while (1);        
	}
	/* The semaphore is used by the display task
	* display task pushes the data out to console/gui composer
	*/
	if(0 != sem_init(&displaySem,0,0))
	{
		/* sem_init() failed */
		Display_print0(display, 0, 0, "displaySem Semaphore creation failed");
		while (1);        
	}
	pthread_attr_init(&pAttrs);
	/* Set priority and stack size attributes */
	pthread_attr_setstacksize(&pAttrs, BMI_TASK_STACK_SIZE);
	priParam.sched_priority = 2;
	pthread_attr_setschedparam(&pAttrs, &priParam);
	retc = pthread_create(&cbTask, &pAttrs, bmiInterruptHandlerTask, NULL);
	if (retc != 0) {
		/* pthread_create() failed */
		Display_print0(display, 0, 0, "BMI interrupt handler Task creation"
				"failed");
		while (1);
	}
	
	pthread_attr_init(&pAttrs);
	/* Set priority and stack size attributes */
	pthread_attr_setstacksize(&pAttrs, DISPLAYTASKSTACKSIZE);
	retc = pthread_create(&dispTask, &pAttrs, displayTask, NULL);
	if (retc != 0) {
		/* pthread_create() failed */
		Display_print0(display, 0, 0, "BMI interrupt handler Task creation"
				"failed");
		while (1);
	}
	com_rslt += bmi160_init(&s_bmi160);
	
	/*reset the bmi160  interrupt engine, FIFO*/
	bmi160_set_command_register(0xB0);
	bmi160_set_command_register(0xB1);
	
	/**** standard 9Dof with FIFO output****/
	com_rslt += bmi160_config_running_mode(STANDARD_UI_9DOF_FIFO);
	GPIO_setCallback(CONFIG_GPIO_BMI160_INT1, &bmi160Callback);
	GPIO_clearInt(CONFIG_GPIO_BMI160_INT1);
	GPIO_enableInt(CONFIG_GPIO_BMI160_INT1);
	
	pthread_attr_destroy(&pAttrs);
	
	return com_rslt;
}

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
BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(
u8 v_running_mode_u8)
{
	/* Variable used for get the status of mag interface*/
	u8 v_mag_interface_u8 = BMI160_INIT_VALUE;
	u8 v_bmm_chip_id_u8 = BMI160_INIT_VALUE;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;
		/* Configure the gyro sleep setting based on your need*/
	if (v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) {
		gyr_setting. sleep_trigger = BMI160_SLEEP_TRIGGER;
		gyr_setting. wakeup_trigger = BMI160_WAKEUP_TRIGGER;
		gyr_setting. sleep_state = BMI160_SLEEP_STATE;
		gyr_setting. wakeup_int = BMI160_WAKEUP_INTR;
	}
	/* The below code used for enable and
	disable the secondary mag interface*/
	com_rslt = bmi160_get_if_mode(&v_mag_interface_u8);
	if (((v_running_mode_u8 == STANDARD_UI_IMU_FIFO) ||
	(v_running_mode_u8 == STANDARD_UI_IMU) ||
	(v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) ||
	(v_running_mode_u8 == APPLICATION_NAVIGATION) ||
	(v_running_mode_u8 == ACCEL_PEDOMETER) ||
	(v_running_mode_u8 == APPLICATION_REMOTE_CONTROL) ||
	(v_running_mode_u8 == APPLICATION_INDOOR_NAVIGATION))
	&& (v_mag_interface_u8 == BMI160_MAG_INTERFACE_ON_PRIMARY_ON)) {
		com_rslt +=
		bmi160_set_bmm150_mag_and_secondary_if_power_mode(
		MAG_SUSPEND_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		com_rslt += bmi160_set_if_mode(
		BMI160_MAG_INTERFACE_OFF_PRIMARY_ON);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	}
	if (((v_running_mode_u8 == STANDARD_UI_9DOF_FIFO)
		|| (v_running_mode_u8 == APPLICATION_HEAD_TRACKING) ||
		(v_running_mode_u8 == APPLICATION_NAVIGATION)) &&
		(v_mag_interface_u8 == BMI160_MAG_INTERFACE_OFF_PRIMARY_ON)) {
			/* Init the magnetometer */
			com_rslt += bmi160_bmm150_mag_interface_init(
			&v_bmm_chip_id_u8);
			/* bmi160_delay_ms in ms*/
			s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	switch (v_running_mode_u8) {
	case STANDARD_UI_9DOF_FIFO:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO mag*/
		com_rslt += bmi160_set_fifo_mag_enable(FIFO_MAG_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		com_rslt += bmi160_interrupt_configuration();
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		//BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark*/
        bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, &accel_off_x,
                &accel_off_y, &accel_off_z);
		bmi160_set_foc_gyro_enable(0x01, &gyro_off_x, &gyro_off_y, &gyro_off_z);

		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		//BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INTR1_MAP_FIFO_WM,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt1*/
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(BMI160_FIFO_WM_ENABLE,
		BMI160_ENABLE);
#if 0
		//com_rslt=bmi160_set_command_register(0xB0);
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_full(BMI160_INTR2_MAP_FIFO_FULL,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
				BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
	    com_rslt += bmi160_set_intr_enable_1(BMI160_FIFO_FULL_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
				BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
#endif

		//com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
		//&header_data);
				/* read the FIFO data*/
		//com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
		//&header_data);
	break;
	case STANDARD_UI_IMU_FIFO:
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		/* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(BMI160_FIFO_WM_ENABLE,
		BMI160_ENABLE);
		/* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_INIT_VALUE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(BMI160_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the fifo water mark as 10*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
		&header_data);
	break;
	case STANDARD_UI_IMU:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case STANDARD_UI_ADVANCEPOWERSAVE:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/

		/* Enable any motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable any motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_0(BMI160_ANY_MOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_X_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Y_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Enable no motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_2(BMI160_NOMOTION_Z_ENABLE,
		BMI160_ENABLE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep trigger*/
		com_rslt += bmi160_set_gyro_sleep_trigger(
		gyr_setting.sleep_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup trigger*/
		com_rslt += bmi160_set_gyro_wakeup_trigger(
		gyr_setting.wakeup_trigger);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro sleep state*/
		com_rslt += bmi160_set_gyro_sleep_state(
		gyr_setting.sleep_state);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set the gyro wakeup interrupt*/
		com_rslt += bmi160_set_gyro_wakeup_intr(gyr_setting.wakeup_int);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case ACCEL_PEDOMETER:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_LOWPOWER);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as SUSPEND write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSR4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 25Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ,
			BMI160_ACCEL_OSR4_AVG1);
		/* 10 not available*/
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_HEAD_TRACKING:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 1600Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
		BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 1600Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
		BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ, BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data */
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
	break;
	case APPLICATION_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		/* read mag data*/
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
	break;
	case APPLICATION_REMOTE_CONTROL:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
				/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
		BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data */
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
	break;
	case APPLICATION_INDOOR_NAVIGATION:
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* Set the gryo bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
			BMI160_GYRO_OUTPUT_DATA_RATE_400HZ);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
			BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ,
			BMI160_ACCEL_OSR4_AVG1);
		s_bmi160.delay_msec(
			BMI160_GEN_READ_WRITE_DELAY);/* bmi160_delay_ms in ms*/
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
		break;
	}

	return com_rslt;

}
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
BMI160_RETURN_FUNCTION_TYPE bmi160_interrupt_configuration(void)
{
	/* This variable used for provide the communication
	results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	/* Configure the in/out control of interrupt1*/
	com_rslt = bmi160_set_output_enable(BMI160_INTR1_OUTPUT_TYPE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the in/out control of interrupt2*/
	com_rslt += bmi160_set_output_enable(BMI160_INTR2_OUTPUT_TYPE,
	BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt1 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_INTR1_LEVEL,
	BMI160_LEVEL_HIGH);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	/* Configure the interrupt2 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_INTR2_LEVEL,
	BMI160_LEVEL_HIGH);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	return com_rslt;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *            will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *            which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BMI160_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	/* Please take the below function as your reference
	* for read the data using I2C communication
	* add your I2C rad function here.
	* "IERROR = I2C_WRITE_READ_STRING(
	*  DEV_ADDR, ARRAY, ARRAY, C_BMI160_ONE_U8X, CNT)"
	* iError is an return value of SPI write function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	*/
	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = cnt;
	i2cTransaction.slaveAddress = dev_addr;
	
	if (!I2C_transfer(i2cHandle,&i2cTransaction))
	{
		ierror = -1;
	}
	
	return (s8)ierror;
}

/*	\Brief: The function is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*            will data is going to be read
*	\param reg_data : This data read from the sensor,
*            which is hold in an array
*	\param cnt : The no of byte of data to be read
*/
s8 BMI160_I2C_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	/* Please take the below function as your reference
	* for read the data using I2C communication
	* add your I2C rad function here.
	* "IERROR = I2C_WRITE_READ_STRING(
	*  DEV_ADDR, ARRAY, ARRAY, C_BMI160_ONE_U8X, CNT)"
	* iError is an return value of SPI write function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	*/
	i2cTransaction.writeBuf = &reg_addr;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = reg_data;
	i2cTransaction.readCount = cnt;
	i2cTransaction.slaveAddress = dev_addr;
	
	if (!I2C_transfer(i2cHandle,&i2cTransaction))
	{
		ierror = -1;
	}
	
	return (s8)ierror;
}

 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *              will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMI160_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 ierror = BMI160_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMI160_INIT_VALUE;
	I2C_Transaction i2cTransaction;
	
	array[BMI160_INIT_VALUE] = reg_addr;
	for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++)
		array[stringpos + BMI160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data +
				stringpos);
	
	i2cTransaction.writeBuf = array;
	i2cTransaction.writeCount = cnt + 1;
	i2cTransaction.readCount = 0;
	i2cTransaction.slaveAddress = dev_addr;
	
	/* If transaction success */
	if (!I2C_transfer(i2cHandle, &i2cTransaction)) {
		ierror = -1;
	}
	
	return (s8)ierror;
	
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+C_BMI160_ONE_U8X)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMI160_INIT_VALUE
	* and FAILURE defined as -C_BMI160_ONE_U8X
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+C_BMI160_ONE_U8X operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
}

/*	Brief : The delay routine
 *	\param : delay in ms
*/
static void BMI160_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	usleep(msek * 1000);
}
