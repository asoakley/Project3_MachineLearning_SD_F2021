void setupAudioSampling() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);      // wait for the serial port to open
  Serial.println("Multitasking active.");
  digitalWrite(LED_FR_PIN, HIGH);
  digitalWrite(LED_FL_PIN, HIGH);
  // Fill classification buffer with audio data
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 1) {
    // Analog read mic pin
    audio = analogRead(MIC_OUT);
    // Add audio data to buffer
    buffer[i] = audio;
    delay(SAMPLE_PERIOD);
  }
}

void loopAudioSampling() {
  float queue[SAMPLE_SIZE] = { 0 };

  // Load in queue of audio data
  for (int i = 0; i < SAMPLE_SIZE; i += 1) {
    audio = analogRead(MIC_OUT);
    queue[i] = audio;
    delay(SAMPLE_PERIOD);
  }

  // Shift queue values into buffer
  int j = 0;
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - SAMPLE_SIZE; i += 1) {
    buffer[i] = buffer[i+SAMPLE_SIZE];
  }
  for (int i = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - SAMPLE_SIZE; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 1) {
    buffer[i] = queue[j];
    j += 1;
  }
}
