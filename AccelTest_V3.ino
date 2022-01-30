
#include <SimpleRSLK.h>    
#include <BMI160Gen.h>

#define IDLE_STATE          0
#define FORWARD_REVERSE     1
#define OSCILLATE           2

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 57;

int timerCounter = 0; /*Counter for the motor state machine*/
int motorState = IDLE_STATE;

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


  /* Wait for button press*/
  waitBtnPressed();

  /* Wait two seconds before starting */
  delay(2000);
  digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);

  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS,15);
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

  delay(100);
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

void waitBtnPressed() {
  while(1) {
    if (digitalRead(LP_S1_PIN) == 0){
      motorState = FORWARD_REVERSE;
      break;
    }
    if (digitalRead(LP_S2_PIN) == 0){
      motorState = OSCILLATE;
      break;
    }
    digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
        delay(500);
        digitalWrite(LP_RGB_LED_GREEN_PIN, LOW);
        delay(500);
  }
}

void timerFunction(){
  static char toggle = 0;
  digitalWrite(LP_RGB_LED_RED_PIN, !digitalRead(LP_RGB_LED_RED_PIN));
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
