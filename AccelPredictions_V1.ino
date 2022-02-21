// Change this include to match the name of your imported Edge Impulse Arduino library
#include <tutorial_continuous_motion_recognition_inference.h>

// RSLK and accelerometer libraries
#include <SimpleRSLK.h>
#include <BMI160Gen.h>

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 5;

// Constant defines
// #define CONVERT_G_TO_MS2    9.810f

// Private variables
// static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Allocate a buffer here for the values we'll read from the IMU
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

#define SAMPLE_PERIOD    25 // 0.025 seconds, 40 Hz
#define MOTOR_PERIOD    500 // 0.5 seconds

int counter = 0;

void setup() {
  
  setupRSLK();
  disableMotor(BOTH_MOTORS);

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

  //attachInterrupt(digitalPinToInterrupt(LP_S1_PIN),SW1_ISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(LP_S2_PIN),SW2_ISR, RISING);
  
  // By default MSP432 has analogRead() set to 10 bits.
  // This Sketch assumes 12 bits. Uncomment to line below to set analogRead()
  // to 12 bit resolution for MSP432.
  //analogReadResolution(12);

  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // Initialize accelerometer
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
  //digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }
}

void waitBtnPressed() {
  while(1) {
    if (digitalRead(LEFT_BTN) == 0) break;
    if (digitalRead(RIGHT_BTN) == 0) break;
    delay(100);
  }
}

void led(){
  digitalWrite(LED_FR_PIN, !digitalRead(LED_FR_PIN));
  digitalWrite(LED_FL_PIN, !digitalRead(LED_FL_PIN));
  digitalWrite(LED_BR_PIN, !digitalRead(LED_BR_PIN));
  digitalWrite(LED_BR_PIN, !digitalRead(LED_BL_PIN));
}

void getAcceleration() {
  int xRaw, yRaw, zRaw;
  float x, y, z;
  updateMotors();
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
    //uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
    BMI160.readAccelerometer(xRaw, yRaw, zRaw);
    x = convertRawAccel(xRaw);
    y = convertRawAccel(yRaw);
    z = convertRawAccel(zRaw);
    // Add accelerometer data to buffer
    buffer[i + 0] = x;
    buffer[i + 1] = y;
    buffer[i + 2] = z;
    //delayMicroseconds(next_tick - micros());
    delay(SAMPLE_PERIOD);
    counter++;
    if(counter > (MOTOR_PERIOD / SAMPLE_PERIOD) - 1){  // If motor period is 500 and sample period is 100, motors will change direction every .5 seconds
      updateMotors();
      counter = 0;
    }
  }
}

float convertRawAccel(int gRaw) {
  // since we are using 4G range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767
  float g = (gRaw * 40) / 32768.0;
  return g;
}

void updateMotors(){
  static char toggle = 0;
  
  setMotorSpeed(BOTH_MOTORS,15);
  //digitalWrite(LP_RGB_LED_RED_PIN, !digitalRead(LP_RGB_LED_RED_PIN));

  if(toggle){
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    }
  else{
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    }
  toggle ^= 1;
}

void predict() {
  // Turn off LEDs
  led();
  digitalWrite(LP_RGB_LED_BLUE_PIN, LOW);
  digitalWrite(LP_RGB_LED_BLUE_PIN, LOW);
  digitalWrite(LP_RGB_LED_BLUE_PIN, LOW);
  delay(500);

  // Prepare for program to start
  Serial.println("\nStarting inferencing in 3 seconds...");
  for (i=0; i<3; i++){
    led();
    delay(500);
    led();
    delay(500);
  }

  // get accelerometer data
  Serial.println("Sampling...");
  getAcceleration();
  disableMotor(BOTH_MOTORS);

  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // Print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

///////////////////// Display predictions with RGB LED /////////////////////
    if (result.classification[ix].label == "Table"){
      digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);
    }
    else {
      digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
    }
////////////////////////////////////////////////////////////////////////////
  }
}

void loop(){
  // Robot is waiting to start
  led();
  waitBtnPressed();
  
  // Run accelerometer ML algorithm
  predict();
}
