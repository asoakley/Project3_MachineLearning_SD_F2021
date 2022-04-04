/*
Version 2 of Audio Testing Code
2/22/2022
*/

#include <SimpleRSLK.h>    
#define MIC_PWR         5   // RLSK Pin P4.1, BoosterPack(Energia) Pin J1-5
#define MIC_OUT         6   // RSLK Pin P4.3, BoosterPack(Energia) Pin J1-6
#define SAMPLE_PERIOD   1 // 1ms, 1000 Hz

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

  pinMode(MIC_PWR, OUTPUT);     
  digitalWrite(MIC_PWR, HIGH);  // Provide power to the mic
  pinMode(MIC_OUT, INPUT);    //Set the output of the mic to an analog input

  analogReadResolution(14); //ADC has 14 bits of resolution
  
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // Wait two seconds before starting
  delay(2000);
  digitalWrite(LP_RGB_LED_BLUE_PIN, HIGH);
}

void loop() {
  int audio;

  //Read the analog voltage of the mic output
  audio = analogRead(MIC_OUT);

  if(audio >= 8800 || audio <= 7800) digitalWrite(LP_RED_LED_PIN, HIGH);
  else digitalWrite(LP_RED_LED_PIN, LOW);
  
 //Send the data over UART
  Serial.println(audio); 

  // Delay the loop to control audio sampling rate
  delay(SAMPLE_PERIOD);
}
