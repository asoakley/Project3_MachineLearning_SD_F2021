// Change this include to match the name of your imported Edge Impulse Arduino library
#include <TI-ML-Audio-Classification_inferencing.h>

// RSLK and accelerometer libraries
#include <SimpleRSLK.h>

#define MIC_PWR   5   // RLSK Pin P4.1, BoosterPack(Energia) Pin J1-5
#define MIC_OUT   6   // RSLK Pin P4.3, BoosterPack(Energia) Pin J1-6

// Private variables
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Allocate a buffer here for the values we'll read from the IMU
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

#define SAMPLE_PERIOD    1    // 5 ms, 1 kHz
#define SAMPLE_SIZE      10   // 10 samples of audio data transfered at a time
int audio;

void setupMain() {
  
  setupRSLK();
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS,0);

  // Initialize LED pins as outputs
  pinMode(LED_FR_PIN, OUTPUT); 
  pinMode(LED_FL_PIN, OUTPUT); 
  pinMode(LP_RED_LED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_RED_PIN, OUTPUT);
  pinMode(LP_RGB_LED_BLUE_PIN, OUTPUT); 
  pinMode(LP_RGB_LED_GREEN_PIN, OUTPUT);

  pinMode(MIC_PWR, OUTPUT);
  digitalWrite(MIC_PWR, HIGH); // Provide power to the mic
  pinMode(MIC_OUT, INPUT);     // Set the output of the mic to an analog input
  
  analogReadResolution(14);

  Serial.begin(115200); // initialize Serial communication
  while (!Serial);      // wait for the serial port to open

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 1) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 1 (the one audio pin)\n");
    return;
  }
}

void audioPredict() {
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
///////////////// Edit this section according to your model classes /////////////////
  // Move motors for set time when detecting motion commands
  // If a command is not detected, program immediately runs back through loop
  if (pred == "Go"){
    setMotorSpeed(BOTH_MOTORS,15);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    delay(1000);
  }
  if (pred == "Back"){
    setMotorSpeed(BOTH_MOTORS,15);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    delay(1000);
  }
  if (pred == "Spin"){
    setMotorSpeed(BOTH_MOTORS,15);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    delay(1000);
  }
  setMotorSpeed(BOTH_MOTORS,0);
/////////////////////////////////////////////////////////////////////////////////////
}

void loopMain(){
  // Run audio ML algorithm
  audioPredict();
}
