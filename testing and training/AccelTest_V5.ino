/*
Version 5 of Accelerometer Testing Code
2/22/2022
*/

#include <SimpleRSLK.h>    
#include <BMI160Gen.h>

#define OFF                 0
#define FORWARD_REVERSE     1
#define OSCILLATE           2
#define FORWARD             3
#define NUM_STATES          3
#define SAMPLE_PERIOD       25 // 0.1 seconds, 10 Hz
#define MOTOR_PERIOD        500 // 0.5 seconds

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 5;

int timerCounter = 0; // Counter for the motor state machine
volatile int motorState = OFF;

void setup() {

  setupRSLK();

  // Initialize LED pins as outputs
  pinMode(LED_FR_PIN, OUTPUT); 
  pinMode(LED_FL_PIN, OUTPUT); 
  pinMode(LED_BR_PIN, OUTPUT); 
  pinMode(LED_BL_PIN, OUTPUT);
  pinMode(LP_RED_LED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_RED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_BLUE_PIN, OUTPUT); 
  pinMode(LP_RGB_LED_GREEN_PIN, OUTPUT);

  // Initialize LaunchPad buttons as inputs
  pinMode(LP_S1_PIN, INPUT_PULLUP);
  pinMode(LP_S2_PIN, INPUT_PULLUP);

  // Attach interrupts to the switches
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

  // Wait two seconds before starting
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

  // display comma-separated gyro x/y/z values
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.println();

  // Delay the loop to control audio sampling rate
  delay(SAMPLE_PERIOD);
  timerCounter++;
  if(timerCounter > (MOTOR_PERIOD / SAMPLE_PERIOD) - 1){  // If motor period is 500 and sample period is 100, motors will change direction every .5 seconds
    timerFunction();
    timerCounter = 0;
  }
}

// This function converts raw accel data to m/s^2
float convertRawAccel(int gRaw) {
  // since we are using 4G range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767

  float g = (gRaw * 40) / 32768.0;

  return g;
}

// Interrupt for SW1 press, cycles through motor states with each press
void SW1_ISR(){
  motorState = motorState % NUM_STATES;
  motorState++;
  enableMotor(BOTH_MOTORS);
  if(motorState == FORWARD){
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  }
}

// Interrupt for SW2 press, disables motors
void SW2_ISR(){
  motorState = OFF;
  disableMotor(BOTH_MOTORS);
}

// Frequency of calls to this function is based on MOTOR_PERIOD
void timerFunction(){
  updateMotors();
}

// This function changes the state of the motors based on time and switch inputs 
void updateMotors(){
  static char toggle = 0;
  
  setMotorSpeed(BOTH_MOTORS,15); // Keep motors at constant speed
  digitalWrite(LP_RGB_LED_RED_PIN, !digitalRead(LP_RGB_LED_RED_PIN)); // Toggle RED
  
  switch(motorState){
    case FORWARD_REVERSE:
      if(toggle){
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      }
      else{
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      }
      break;

    case OSCILLATE:
      if(toggle){
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      }
      else{
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      }
      break;
    default: break;
  }
  toggle ^= 1;
}
