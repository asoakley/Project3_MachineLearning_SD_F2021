/*
When connected to the Python Client, this program prints text when SW1 is pressed.
Edit the ssid and password variables to match your network. 
The assigned IP and port printed in the serial terminal need to match those in the Python script
*/

#ifndef __CC3200R1M1RGC__
// Do not include SPI for CC3200 LaunchPad
#include <SPI.h>
#endif
#include <WiFi.h>
#include <SimpleRSLK.h>

// your network name also called SSID
char ssid[] = "The Salty Spitoon";
// your network password
char password[] = "olivezebra142";
// your network key Index number (needed only for WEP)
unsigned char mac[6];
int port = 9099;
int keyIndex = 0;
volatile bool msgRdy = false;

WiFiServer server(port);

void setup() {
  setupRSLK();

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

  // Attach interrupts to the switches
  attachInterrupt(digitalPinToInterrupt(LP_S1_PIN),SW1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LP_S2_PIN),SW2_ISR, RISING);
  
  Serial.begin(115200);      // initialize serial communication
  
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");


  // Print the MAC Address
  WiFi.macAddress(mac);
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);

  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, password);
  //WiFi.begin(ssid);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  
  // you're connected now, so print out the status  
  printWifiStatus();

  Serial.print("Starting webserver on port ");
  Serial.println(port);
  server.begin();                           // start the web server on port xxxx
  Serial.println("Webserver started!");
}

void loop() {
  int i = 0;
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    while (client.connected()) {            // loop while the client's connected
      if(msgRdy){
        client.println("sussy");
        msgRdy = false;
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}

//
//a way to check if one array ends with another array
//
boolean endsWith(char* inString, const char* compString) {
  int compLength = strlen(compString);
  int strLength = strlen(inString);
  
  //compare the last "compLength" values of the inString
  int i;
  for (i = 0; i < compLength; i++) {
    char a = inString[(strLength - 1) - i];
    char b = compString[(compLength - 1) - i];
    if (a != b) {
      return false;
    }
  }
  return true;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  //long rssi = WiFi.RSSI();
  
  //Serial.print("signal strength (RSSI):");
  //Serial.print(rssi);
  //Serial.println(" dBm");
}

// Interrupt for SW1 press
void SW1_ISR(){
  msgRdy = true;
}

// Interrupt for SW2 press
void SW2_ISR(){
  msgRdy = false;
}
