## Example Summary

Sample application to read 9 DoF(Degrees of Freedom) from (BMI160(inertial measurement unit) + BMM150(GeoMagnetic Sensor). The BMM150 in this example has been configured as slave to the BMI160.
So single driver for BMI160 sensor also reads data from the  BMM150 .

## Peripherals & Pin Assignments

SysConfig generates the driver configurations into the __ti_drivers_config.c__
and __ti_drivers_config.h__ files. Information on pins and resources used
is present in both generated files. The SysConfig user interface can also be
utilized to determine pins and resources used.

Pin assignments for BOOSTXL-SENSORS BoosterPack
* `CONFIG_I2C_BMI` - I2C peripheral instance used to communicate with BMI160 Sensor + BMM150 Sensor(BMM150 interfaced as slave to BMI160).
* `CONFIG_GPIO_BMI160_INT1` - BMI160 interrupt pin1


## BoosterPacks, Board Resources & Jumper Settings

This example requires a [__BOOSTXL-SENSORS BoosterPack__](http://www.ti.com/tool/BOOSTXL-SENSORS).

> If you're using an IDE (such as CCS or IAR), please refer to Board.html in your project
directory for resources used and board-specific jumper settings. Otherwise, you can find
Board.html in the directory &lt;PLUGIN_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

## Example Usage

* Connect the BOOSTXL-SENSORS BoosterPack before powering the hardware.

* Open a serial session (e.g. [`PuTTY`](http://www.putty.org/ "PuTTY's Homepage"),teraterm, CCS terminal, etc.) to the appropriate COM port.
    * The COM port can be determined via Device Manager in Windows or via `ls /dev/tty*` in Linux.

The connection will have the following settings:
```
    Baud-rate:     115200
    Data bits:          8
    Stop bits:          1
    Parity:          None
    Flow Control:    None
```

* Run the example.

* The example collects the samples from BMI160 driver in interrupt FIFO mode and displays
average value on the Serial Console as below.On changing the orientation of the LaunchPad the gyro accelo and magno values should change accordingly.
```
    Starting the i2cbmi160 sensor example...

    accelo: x = 1, y = -23,z = 8207

    gyro  : x = 0, y = 0,z = 0

    magno : x = 138, y = 203,z = 0

    accelo: x = 9, y = -28,z = 8207

    gyro  : x = -1, y = 0,z = 1

    magno : x = 287, y = 416,z = 0

```

## Application Design Details

This application uses three threads:

`mainThread` - performs the following actions:

1. Opens and initializes an I2C Driver.

2. Creates two threads bmiInterruptHandlerTask, displayTask

3. reset the bmi160  interrupt engine, FIFO

4. Initializes the BMI160 driver to operate in STANDARD_UI_9DOF_FIFO mode

5. Sets Callback for FIFO Water Mark Interrupt.

`bmiInterruptHandlerTask` - performs the following actions:

1. the bmiInterruptHandlerTask gets unblocked whenever there is a FIFO water mark interrupt.(Water mark is set for 512).

2. Reads the FIFO and averages the values read.

3. Unblocks the Display task.

`displayTask` - performs the following actions:

1. This task prints the values of accelo, gyro, magno on serial console.
