#include <SimpleRSLK.h> 

#define MIC_PWR   5
#define MIC_OUT   6


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
  pinMode(MIC_PWR, OUTPUT);
  digitalWrite(MIC_PWR, HIGH);  /*Provide power to the mic*/
  pinMode(MIC_OUT, INPUT);    /*Set the output of the mic to an analog input*/

  analogReadResolution(14); /*ADC has 14 bits of resolution*/
  
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

}

void loop() {
  int audio;

  /*Read the analog voltage of the mic output*/
  audio = analogRead(MIC_OUT);

 /*Send the data over UART*/
  Serial.print(audio); 
  Serial.println();

  delay(5); /*Delay of 5 ms for 200 Hz sampling rate*/

}
