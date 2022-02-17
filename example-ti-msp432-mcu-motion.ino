// Change this include to match the name of your imported Edge Impulse Arduino library
#include <tutorial_continuous_motion_recognition_inference.h>

// Include application, user and local libraries
#include "SPI.h"
#define HX8353E // HX8353E K35_SPI
#include "Screen_HX8353E.h"
Screen_HX8353E myScreen;

const int xpin = 23; // x-axis of the accelerometer
const int ypin = 24; // y-axis
const int zpin = 25; // z-axis

int currentOrientation = 4;
int newOrientation = 0;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.810f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Allocate a buffer here for the values we'll read from the IMU
float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

void setup() {
  // By default MSP432 has analogRead() set to 10 bits.
  // This Sketch assumes 12 bits. Uncomment to line below to set analogRead()
  // to 12 bit resolution for MSP432.
  //analogReadResolution(12);
  Serial.begin(115200);

  // init lcd screen
  delay(100);

  myScreen.begin();
  Serial.println(myScreen.WhoAmI());
  Serial.print(myScreen.screenSizeX(), DEC);
  Serial.print("x");
  Serial.println(myScreen.screenSizeY(), DEC);

  myScreen.setFontSize(3);
  myScreen.clear(whiteColour);
  currentOrientation = myScreen.getOrientation();
  myScreen.setOrientation(currentOrientation);

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }
}

void getAcceleration() {
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
    float x = (float)analogRead(xpin);
    float y = (float)analogRead(ypin);
    float z = (float)analogRead(zpin);
    // Add accelerometer data to buffer
    buffer[i + 0] = x;
    buffer[i + 1] = y;
    buffer[i + 2] = z;
    /* Comment out the previous 3 lines and uncomment the next 3 lines if you
       are using the data provided by the cloned "Tutorial: continuous motion
       recognition" project (this code is used to convert the IMU data to 
       match the Edge Impulse model scaling) */
    //buffer[i + 0] = (x - 511.0f) / (1023.0f - 511.0f) * (9.810f * 2.0f);
    //buffer[i + 1] = (y - 511.0f) / (1023.0f - 511.0f) * (9.810f * 2.0f);
    //buffer[i + 2] = (z - 511.0f) / (1023.0f - 511.0f) * (9.810f * 2.0f);
    delayMicroseconds(next_tick - micros());
  }
}

void loop() {
  Serial.println("\nStarting inferencing in 2 seconds...");
  myScreen.gText(2, 10, "Sampling in 2 sec...", blackColour, whiteColour);

  delay(2000);

  Serial.println("Sampling...");
  myScreen.gText(2, 10, "Sampling...                ", blackColour, whiteColour);

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

  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }

  // print the predictions
  myScreen.gText(2, 30, "Predictions:", blackColour, whiteColour);
  myScreen.gText(2, 40, "DSP: " + String(result.timing.dsp) + " ms   ", blackColour, whiteColour);
  myScreen.gText(2, 50, "Classification: " + String(result.timing.classification) + " ms   ", blackColour, whiteColour);
  myScreen.gText(2, 60, "Anomaly: " + String(result.timing.anomaly) + " ms   ", blackColour, whiteColour);

  int currentIdx = 80;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    myScreen.gText(10, currentIdx, String(result.classification[ix].label) + ": " + String(result.classification[ix].value), redColour, whiteColour);
    currentIdx += 10;
  }
}

