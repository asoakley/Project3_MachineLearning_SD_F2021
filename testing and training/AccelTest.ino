/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read ayroscope data
*/
      
#include <BMI160Gen.h>

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 57;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 4 G's
  BMI160.setAccelerometerRange(4);
  Serial.println("Initializing IMU device...done.");
}

void loop() {
  int axRaw, ayRaw, azRaw;         // raw ayro values
  float ax, ay, az;

  // read raw ayro measurements from device
  BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

  // convert the raw ayro data to degrees/second
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);

  // display tab-separated ayro x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();

  delay(100);
}

float convertRawAccel(int aRaw) {
  // since we are using 4 G's
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767

  float a = (aRaw * 40) / 32768.0;

  return a;
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
