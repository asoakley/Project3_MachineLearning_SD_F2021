/*
Version 2 of Audio Testing Code
2/22/2022
*/

#include <SimpleRSLK.h>    

#define OFF                 0
#define FORWARD_REVERSE     1
#define OSCILLATE           2
#define FORWARD             3
#define NUM_STATES          3
#define SAMPLE_PERIOD       5 // 5ms for 200 Hz sampling rate
#define MOTOR_PERIOD        500 // 0.5 seconds
#define MIC_PWR   5   // RLSK Pin P4.1, BoosterPack(Energia) Pin J1-5
#define MIC_OUT   6   // RSLK Pin P4.3, BoosterPack(Energia) Pin J1-6

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

  pinMode(MIC_PWR, OUTPUT);     
  digitalWrite(MIC_PWR, HIGH);  // Provide power to the mic
  pinMode(MIC_OUT, INPUT);    //Set the output of the mic to an analog input

  analogReadResolution(14); //ADC has 14 bits of resolution
  
  // Initialize LaunchPad buttons as inputs
  pinMode(LP_S1_PIN, INPUT_PULLUP);
  pinMode(LP_S2_PIN, INPUT_PULLUP);

  // Attach interrupts to the switches
  attachInterrupt(digitalPinToInterrupt(LP_S1_PIN),SW1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LP_S2_PIN),SW2_ISR, RISING);
  
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

 //Send the data over UART
  Serial.print(audio); 
  Serial.println();

  // Delay the loop to control audio sampling rate
  delay(SAMPLE_PERIOD);
  timerCounter++;
  if(timerCounter > (MOTOR_PERIOD / SAMPLE_PERIOD) - 1){  // If motor period is 500 and sample period is 100, motors will change direction every .5 seconds
    timerFunction();  // Calls motor update function
    timerCounter = 0;
  }
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
    default: break;
  }
  toggle ^= 1;
}
