/*
Version 4 of Accelerometer Testing Code
Switch 1 Enables Motors (Forward)
Switch 2 Disables Motors
Accelerometer Sampling Continuously

*/

#include <SimpleRSLK.h>    
#include <BMI160Gen.h>

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 5;

int timerCounter = 0; /*Counter for the motor state machine*/

void setup() {

  setupRSLK();

  /* Initialize LED pins as outputs */
  pinMode(LED_FR_PIN, OUTPUT); 
  pinMode(LED_FL_PIN, OUTPUT); 
  pinMode(LED_BR_PIN, OUTPUT); 
  pinMode(LED_BL_PIN, OUTPUT);
  pinMode(LP_RED_LED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_RED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_BLUE_PIN, OUTPUT); 
  pinMode(LP_RGB_LED_GREEN_PIN, OUTPUT);

  /* Initialize LaunchPad buttons as inputs */
  pinMode(LP_S1_PIN, INPUT_PULLUP);
  pinMode(LP_S2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LP_S1_PIN),SW1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LP_S2_PIN),SW2_ISR, RISING);
  
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize accelerometer
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

   // Set the accelerometer range to 4G
  BMI160.setAccelerometerRange(4); 
  Serial.println("Initializing IMU device...done.");

  /* Wait two seconds before starting */
  delay(2000);
  digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);

}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw accel values
  float gx, gy, gz;

  // read raw accel measurements from device
  BMI160.readAccelerometer(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawAccel(gxRaw);
  gy = convertRawAccel(gyRaw);
  gz = convertRawAccel(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println();

  delay(25);
  timerCounter++;
  if(timerCounter > 4){
    timerFunction();
    timerCounter = 0;
  }
}

float convertRawAccel(int gRaw) {
  // since we are using 4G range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767

  float g = (gRaw * 40) / 32768.0;

  return g;
}

void SW1_ISR(){
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS,15);
}

void SW2_ISR(){
  disableMotor(BOTH_MOTORS);
}

void timerFunction(){
  
}
