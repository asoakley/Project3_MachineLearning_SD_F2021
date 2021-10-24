/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSP_EXP432P401R
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_MSP_EXP432P401R

#ifndef DeviceFamily_MSP432P401x
#define DeviceFamily_MSP432P401x
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== GPIO ========
 */

/* P4.1, Acc/Gyro Interrupt 1 */
#define CONFIG_GPIO_BMI160_INT1     0

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== I2C ========
 */

/*
 *  SCL: P6.5
 *  SDA: P6.4
 *  Sensors BoosterPack I2C
 */
#define CONFIG_I2C_BMI              0

/* ======== I2C Addresses and Speeds ======== */
#include <ti/drivers/I2C.h>

/* ---- CONFIG_I2C_BMI I2C bus components ---- */

/* BOOSTXL_SENSORS/BMI160_BMM150 address and max speed */
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_BMI160_BMM150_ADDR     (0x69)
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_BMI160_BMM150_MAXSPEED (1000U) /* Kbps */

/* BOOSTXL_SENSORS/BME280 address and max speed */
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_BME280_ADDR     (0x77)
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_BME280_MAXSPEED (3400U) /* Kbps */

/* BOOSTXL_SENSORS/OPT3001 address and max speed */
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_OPT3001_ADDR     (0x47)
#define CONFIG_I2C_BMI_BOOSTXL_SENSORS_OPT3001_MAXSPEED (2600U) /* Kbps */

/* CONFIG_I2C_BMI max speed (supported by all components) */
#define CONFIG_I2C_BMI_MAXSPEED   (1000U) /* Kbps */
#define CONFIG_I2C_BMI_MAXBITRATE ((I2C_BitRate)I2C_1000kHz)


/*
 *  ======== UART ========
 */

/*
 *  TX: P1.3
 *  RX: P1.2
 *  XDS110 UART
 */
#define CONFIG_UART_0               0


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
