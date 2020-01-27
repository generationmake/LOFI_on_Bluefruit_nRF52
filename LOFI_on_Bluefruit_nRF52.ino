/******************************************************************************
  This software emulates a HM-10 bluetooth module and the protocol used by the LOFI robot app, so that
  this app can control a robot using this Arduino MKR

  By now there are only the motor control functions implemented and analog values are sent to app. 
  The rest is decoded but no action connected.
  
  bernhard@generationmake.de
  Original Creation Date: Jan. 11, 2020

  This code is Beerware; if you see me at the local, 
  and you've found our code helpful, please buy us a round!

  The circuit:
  - Arduino MKR WiFi 1010
  - HighPowerMotorFeatherWing
  
  Distributed as-is; no warranty is given.
******************************************************************************/
#include <ArduinoBLE.h>
#include "ifx007t.h"
 
Ifx007t mot1;
Ifx007t mot2;

#define MOT1_1 2
#define MOT1_2 3
#define MOT1_EN 1
#define MOT2_1 4
#define MOT2_2 5
#define MOT2_EN 0

const long interval = 100;

unsigned long previousMillis = 0;
unsigned long currentMillis;

BLEService lofiService("0000ffe0-0000-1000-8000-00805f9b34fb"); // BLE LOFI Service UUID of HM-10

// BLE Serial Characteristic - custom 128-bit UUID of HM-10, read and writable by central
BLECharacteristic serialCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", BLERead | BLEWriteWithoutResponse | BLENotify, 20);

const int ledPin = LED_BUILTIN; // pin to use for the LED

void setup() {
  Serial.begin(9600);
//  while (!Serial);  // wait until serial is ready. i. e. computer connected. not useful for standalone robot

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LOFI-Robot");   // use custom name, will be displayed in app
  BLE.setDeviceName("HMsoft-38A3"); // device name is necessary that LOFI control recognizes device
  BLE.setAdvertisedService(lofiService);

  // add the characteristic to the service
  lofiService.addCharacteristic(serialCharacteristic);

  // add service
  BLE.addService(lofiService);

  // start advertising
  BLE.advertise();

  Serial.println("BLE Serial Peripheral");
  mot1.begin(MOT1_1,MOT1_2,MOT1_EN);    // configure motors
  mot2.begin(MOT2_1,MOT2_2,MOT2_EN);
  Serial.println("Motor configured");
  mot1.stop();    // stop motor after configuration
  mot2.stop();
}

void loop() {
  uint8_t received[20];
  char prev_byte=0,current_byte=0;
  int receive_length=0;
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {

// transfer data regularly to the app
      currentMillis = millis();

      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        sending();
      } 
// receive data
      if (serialCharacteristic.written()) {   // check if data was sent by app
        receive_length=serialCharacteristic.readValue(received,20);  // get received string
        Serial.print((char*)received);     // print string to serial
        Serial.print("  ");
        for(int i=0; i<receive_length; i++)  // go through string byte by byte
        {
          Serial.print(i);    // print byte and value of byte to serial
          Serial.print(":");
          Serial.print(received[i]);
          Serial.print("=");
          Serial.print(byte(received[i]));
          Serial.print("  ");

//decode
          current_byte=received[i];
          decode_received(prev_byte,current_byte);
          prev_byte=received[i];
        }
        Serial.println("");
      }
    }
    mot1.stop();    // stop motors when central disconnects
    mot2.stop();

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

//decode received bytes and set outputs
void decode_received(byte prev_byte, byte current_byte)
{
  int pwm_value;

  if (prev_byte == 201) { // Buzzer
    digitalWrite(ledPin, current_byte);
  }
  if (prev_byte == 202) { // Motor 1
    if (current_byte <= 100) pwm_value=(current_byte * 2.55); //calculate pwm value
    else pwm_value=(-((current_byte-100) * 2.55));
    mot1.pwm(pwm_value);    // set pwm value
    Serial.print(pwm_value);  // print pwm value to serial
    Serial.print("!");
  }
  if (prev_byte == 203) { // Motor 2
    if (current_byte <= 100) pwm_value=(current_byte * 2.55);  //calculate pwm value
    else pwm_value=(-((current_byte-100) * 2.55));
    mot2.pwm(pwm_value);    // set pwm value
    Serial.print(pwm_value);  // print pwm value to serial
    Serial.print("!");
  }
  if (prev_byte == 204) { // output 1
  
  }
  if (prev_byte == 205) { // output 2
  
  }
  if (prev_byte == 206) { // output 3
  
  }
  if (prev_byte == 207) { // output 4
  
  }
  if (prev_byte == 208) { // servo output 1
  
  }
  if (prev_byte == 209) { // servo output 2
  
  }
  if (prev_byte == 210) { // servo output 3
  
  }
  if (prev_byte == 211) { // servo output 4
  
  }
  if (prev_byte == 212 && current_byte == 99) {
  
  }
  
}

// function for sending values to the app. just copied from LOFI sample, not modified yet
void sending() {
  byte sendstring[20];
  int analog1 = 0;
  int analog2 = 0;
  int analog3 = 0;
  int analog4 = 0;
  analog1 = analogRead(0)/10.23;
  analog2 = analogRead(1)/10.23;
  analog3 = analogRead(2)/10.23;
  analog4 = analogRead(3)/10.23;
  
//[224, 115, 2, 225, 102, 4, 226, 107, 5, 227, 63, 6]
  sendstring[0]=224;
  sendstring[1]=byte(analog1);
  sendstring[2]=225;
  sendstring[3]=byte(analog2);
  sendstring[4]=226;
  sendstring[5]=byte(analog3);
  sendstring[6]=227;
  sendstring[7]=byte(analog4);
  sendstring[8]=240;
  sendstring[9]=100;
  // last byte "i" character as a delimiter for BT2.0 on Android
  sendstring[10]=105;

  serialCharacteristic.writeValue((const uint8_t*)sendstring,11);
}
