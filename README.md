# Project3_MachineLearning_SD_F2021
Code repository for all TI-RSLK software for Senior Design
CCS contains the program files for Code Composer Studios. It was used in the Fall but turned to Arduino as Tensorflow Lite was no longer supported. This folder contains obsolete code
  AudioTest contains the files for running microphone
  I2CandUART contains program files for the accelerometer in CCS up and running.
  IMUtest contains the program files for testing the  IMU on the sensors board
  LEDtest contains the program files to blink LEDs
  MotorsTest contains the program files to spin the motors
  MotorsTestV2 contains the program files to make patterns for the robot to perform (figure 8, triangle, etc)
  boostxl_sensors_sensorgui_msp432p401r
libraries folder contains the libraries used in Arduino IDE to get the MSP432 microcontroller and the RSLK to function. It also contains some of the Edge Impulse libraries that we have tested
prediction folder contains the Arduino code for running edge impulse models. It contains code for both Audio and Accelerometer Machine Learning.
raw data folder  contains the sample data that we used throughout our trails in different audio/accelerometer models.
test and training folder contains Arduino code to sample and try out different functions of the RSLK MAX
  MultitaskTest contains program files an attempt to run multitasking using Arduino
