/*
Version 1 of the Combined Testing Code
3/3/2022
*/

#include <SimpleRSLK.h>
#include <BMI160Gen.h>

///////////////////////////////////////////////////////////////////
   // Specify the training mode
   // 1 ---> Accelerometer training
   // 0 ---> Audio training
   #define TRAINING_MODE    1 
///////////////////////////////////////////////////////////////////
#if TRAINING_MODE // Accelerometer training mode
  #define SAMPLE_PERIOD       25 // 25ms, 40 Hz (max for accel is 1600 Hz)
#else // Audio training mode
  #define SAMPLE_PERIOD       5 // 5ms, 200 Hz (max for audio is 25 kHz)
#endif
///////////////////////////////////////////////////////////////////

#define OFF                 0
#define FORWARD_REVERSE     1
#define OSCILLATE           2
#define FORWARD             3
#define NUM_STATES          3
#define MOTOR_PERIOD        500 // 0.5 seconds
#define MIC_PWR   5   // RLSK Pin P4.1, BoosterPack(Energia) Pin J1-5
#define MIC_OUT   6   // RSLK Pin P4.3, BoosterPack(Energia) Pin J1-6

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 5;
int timerCounter = 0; // Counter for the motor state machine
volatile int motorState = OFF; // Current state of the motors

void setup() {
  
  setupRSLK();  // Setup the basic functions of the RSLK

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

//////////////////////////////////////////////////////////////////////////////
  if(TRAINING_MODE){
    // initialize accelerometer
    Serial.println("Initializing IMU device...");
    BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
    uint8_t dev_id = BMI160.getDeviceID();
    Serial.print("DEVICE ID: ");
    Serial.println(dev_id, HEX);
  
     // Set the accelerometer range to 4G
    BMI160.setAccelerometerRange(4); 
    Serial.println("Initializing IMU device...done.");
  }
//////////////////////////////////////////////////////////////////////////////
  else{
    // Initialize the microphone
    pinMode(MIC_PWR, OUTPUT);     
    digitalWrite(MIC_PWR, HIGH);  // Provide power to the mic
    pinMode(MIC_OUT, INPUT);    //Set the output of the mic to an analog input
  
    analogReadResolution(14); // ADC has 14 bits of resolution
  }
//////////////////////////////////////////////////////////////////////////////

  // Wait two seconds before starting
  delay(2000);
  digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);
}

void loop() {

  if(TRAINING_MODE)
    AccelProcess();
  else AudioProcess();
  
  // Delay the loop to control audio sampling rate
  delay(SAMPLE_PERIOD);
  timerCounter++;
  if(timerCounter > (MOTOR_PERIOD / SAMPLE_PERIOD) - 1){  // If motor period is 500 and sample period is 100, motors will change direction every .5 seconds
    timerFunction();  // Calls motor update function
    timerCounter = 0;
  }
}

void AudioProcess(){
  int audio;

  //Read the analog voltage of the mic output
  audio = analogRead(MIC_OUT);

  //Send the data over UART
  Serial.print(audio); 
  Serial.println();
}

void AccelProcess(){
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
  static char toggle = 0; // Used for periodic direction changes
  
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
      
    case FORWARD:
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      break;
    default: break;
  }
  toggle ^= 1;
}
