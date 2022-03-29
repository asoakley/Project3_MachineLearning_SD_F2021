// AccelPredictions_V2
// Continuous forward drive accelerometer classification

// Change this include to match the name of your imported Edge Impulse Arduino library
#include <TerrainRecognition_inferencing.h>

// RSLK and accelerometer libraries
#include <SimpleRSLK.h>
#include <BMI160Gen.h>

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 17;

volatile int state = 0;

// Private variables
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Allocate a buffer here for the values we'll read from the IMU
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

#define SAMPLE_PERIOD    20 // 0.02 seconds, 50 Hz

void setup() {
  
  setupRSLK();
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS,0);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);

  // Initialize LED pins as outputs
  pinMode(LED_FR_PIN, OUTPUT); 
  pinMode(LED_FL_PIN, OUTPUT); 
  pinMode(LP_RED_LED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_RED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_BLUE_PIN, OUTPUT); 
  pinMode(LP_RGB_LED_GREEN_PIN, OUTPUT);

  // Initialize LaunchPad buttons as inputs
  pinMode(LP_S1_PIN, INPUT_PULLUP);
  pinMode(LP_S2_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(LP_S1_PIN),SW1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LP_S2_PIN),SW2_ISR, RISING);

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

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }
}

void SW2_ISR(){
  state = !state;
}

void getAcceleration() {
  int xRaw, yRaw, zRaw;
  float x, y, z;
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
    BMI160.readAccelerometer(xRaw, yRaw, zRaw);
    x = convertRawAccel(xRaw);
    y = convertRawAccel(yRaw);
    z = convertRawAccel(zRaw);
    // Add accelerometer data to buffer
    buffer[i + 0] = x;
    buffer[i + 1] = y;
    buffer[i + 2] = z;
    delay(SAMPLE_PERIOD);
  }
}

float convertRawAccel(int gRaw) {
  // since we are using 4G range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767
  float g = (gRaw * 40) / 32768.0;
  return g;
}

void predict() {
  // Get accelerometer data
  Serial.println("Sampling...");
  getAcceleration();

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
  float check = 0.0;
  String pred = "";
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    if (result.classification[ix].value > check) {
      pred = result.classification[ix].label;
      check = result.classification[ix].value;
    }
  }
  
  // Turn off RGB LED
  digitalWrite(LP_RGB_LED_RED_PIN, LOW);
  digitalWrite(LP_RGB_LED_BLUE_PIN, LOW);
  digitalWrite(LP_RGB_LED_GREEN_PIN, LOW);
  
///////////////////// Display predictions with RGB LED /////////////////////
  if (pred == "Carpet"){
    digitalWrite(LP_RGB_LED_RED_PIN, HIGH);
    digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
    digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);
  }
  if (pred == "Turf"){
    digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
  }
  if (pred == "Foam") {
    digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);
  }
////////////////////////////////////////////////////////////////////////////
}

void loop(){
  // Robot is waiting to start
  if (state == 0) {
    digitalWrite(LED_FR_PIN, HIGH);
    digitalWrite(LED_FL_PIN, HIGH);
    digitalWrite(LP_RGB_LED_RED_PIN, LOW);
    digitalWrite(LP_RGB_LED_BLUE_PIN, LOW);
    digitalWrite(LP_RGB_LED_GREEN_PIN, LOW);
    setMotorSpeed(BOTH_MOTORS,0);
  }
  
  // Run accelerometer ML algorithm
  if (state == 1) {
    digitalWrite(LED_FR_PIN, LOW);
    digitalWrite(LED_FL_PIN, LOW);
    setMotorSpeed(BOTH_MOTORS,15);
    predict();
  }
}
