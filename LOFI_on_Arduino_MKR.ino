/******************************************************************************
  This software emulates a HM-10 bluetooth module and the protocol used by the LOFI robot app, so that
  this app can control a robot using this Arduino MKR

  By now there are only the motor control functions implemented. No analog in, no servo
  
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

const long interval = 5000;

unsigned long previousMillis = 0;
unsigned long currentMillis;

//int str[20];
int str;
BLEService lofiService("0000ffe0-0000-1000-8000-00805f9b34fb"); // BLE LOFI Service UUID of HM-10

// BLE Serial Characteristic - custom 128-bit UUID of HM-10, read and writable by central
BLEStringCharacteristic serialCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", BLERead | BLEWriteWithoutResponse | BLENotify, 20);

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
  String received;
  char prev_byte=0,current_byte=0;
  int pwm_value;
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
/*
// transfer data regularly to the app. not used yet
      currentMillis = millis();

      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
  //      sending();
      } */
      if (serialCharacteristic.written()) {   // check if data was sent by app
        received=serialCharacteristic.value();  // get received string
        Serial.print(received.c_str());     // print string to serial
        Serial.print("  ");
        for(int i=0; i<received.length(); i++)  // go through strin byte by byte
        {
          Serial.print(i);    // print byte and value of byte to serial
          Serial.print(":");
          Serial.print(received.charAt(i));
          Serial.print("=");
          Serial.print(byte(received.charAt(i)));
          Serial.print("  ");

//decode
          current_byte=received.charAt(i);
          if (prev_byte == 202) { // Motor A
            if (current_byte <= 100) pwm_value=(current_byte * 2.55); //calculate pwm value
            else pwm_value=(-((current_byte-100) * 2.55));
            mot1.pwm(pwm_value);    // set pwm value
            Serial.print(pwm_value);  // print pwm value to serial
            Serial.print("!");
          }
          if (prev_byte == 203) { // Motor B
            if (current_byte <= 100) pwm_value=(current_byte * 2.55);  //calculate pwm value
            else pwm_value=(-((current_byte-100) * 2.55));
            mot2.pwm(pwm_value);    // set pwm value
            Serial.print(pwm_value);  // print pwm value to serial
            Serial.print("!");
          }
          prev_byte=received.charAt(i);
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

// function for sending values to the app. just copied from LOFI sample, not modified yet
void sending() {
  int analog1 = 0;
  int analog2 = 0;
  int analog3 = 0;
  int analog4 = 0;
  analog1 = analogRead(0)/10.23;
  analog2 = analogRead(1)/10.23;
  analog3 = analogRead(2)/10.23;
  analog4 = analogRead(3)/10.23;
  
//[224, 115, 2, 225, 102, 4, 226, 107, 5, 227, 63, 6]
  serialCharacteristic.writeValue("test");
/*  serialCharacteristic.writeValue(224);
  serialCharacteristic.writeValue(byte(analog1));
  serialCharacteristic.writeValue(225);
  serialCharacteristic.writeValue(byte(analog2));
  serialCharacteristic.writeValue(226);
  serialCharacteristic.writeValue(byte(analog3));
  serialCharacteristic.writeValue(227);
  serialCharacteristic.writeValue(byte(analog4));

  //odleglosc();

  serialCharacteristic.writeValue(240);
  serialCharacteristic.writeValue(100);
  // last byte "i" character as a delimiter for BT2.0 on Android
  serialCharacteristic.writeValue(105);
 */
}
