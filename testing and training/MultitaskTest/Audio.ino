#include <SimpleRSLK.h>

#define  MIC_PWR   5   // RLSK Pin P4.1, BoosterPack(Energia) Pin J1-5
#define  MIC_OUT   6   // RSLK Pin P4.3, BoosterPack(Energia) Pin J1-6
#define MIC_SAMPLE_PERIOD       5 // 5ms, 200 Hz

void setupAudio() {
  Serial.begin(115200);
  while (!Serial);    // wait for the serial port to open

  // Initialize the microphone
  pinMode(MIC_PWR, OUTPUT);     
  digitalWrite(MIC_PWR, HIGH);  // Provide power to the mic
  pinMode(MIC_OUT, INPUT);    //Set the output of the mic to an analog input
  
  analogReadResolution(14); // ADC has 14 bits of resolution
}

void loopAudio() {
  int audio;
  //Read the analog voltage of the mic output
  audio = analogRead(MIC_OUT);
  if(audio>= 8800 || audio <= 7800) digitalWrite(LP_RED_LED_PIN, HIGH);
  else digitalWrite(LP_RED_LED_PIN, LOW);
  //Send the data over UART
  Serial.println(audio); 
  delay(MIC_SAMPLE_PERIOD);
}
