#include "MultiMap.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

//__________GPS_____________________________________________

static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;

int satAmount = 5;
int minGPSSpeed = 10;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//________PINS________________________________________________


const int inputPin = 2; // Input Pin ofr Speed Signal
const int analogOutPin = 9; // Analog output pin that the Speedo is attached to

//__________VARIABLES____________________________________________

boolean DEBUG = false;


volatile  unsigned long startTime = 0;
volatile  unsigned long stopTime = 0;
volatile bool newPulseDurationAvailable = false;
volatile bool newPulse = false;

unsigned long previousMicros_InputRead = 0;
unsigned long previousMicros_Speedo = 0;
long inputDuration = 2000000;        // store Speed Input Duration

int sensorValue = 0;        // value read from the pot
long outputValue = 0;        // value output to the PWM (analog out)
long speedKmH = 0;

//_________ SPEEDO NEEDLE CALIBRATION VALUES_____________________________________________

unsigned long minSpeed = 150000;

long speedoIn[] =  {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140};
long speedoOut[] = {200000, 119900, 74730, 46510, 36450, 29110, 24730, 20960, 18250, 16000, 14527, 13361, 12344, 11384, 10567, 9815};

//_________VSS PULSE CALIBRATION VALUES_________________________________________________

//long vssIn[] = {50000, 11909, 7407, 4065, 3604, 2901, 2407, 2009, 1802, 1600, 1405, 1303, 1203, 1103, 1005, 980, 0};
//long vssOut[] =  {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};



long vssIn[] = {5000, 5294, 5625, 6000, 6428, 6923, 7500, 8181, 9000, 10000, 11250, 12850, 15000, 18000, 22500, 30000, 45000, 85800, 244200, 500000};
long vssOut[] =  {180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 0};

//_______________SETUP___________________________________________________

void setup() {
  // initialize serial communications at 9600 bps:
  if (DEBUG) {
    Serial.begin(115200);
  }
  pinMode(analogOutPin, OUTPUT);    // sets the digital pin 9 as output
  digitalWrite(analogOutPin, HIGH);

  pinMode(inputPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(inputPin), pulseLength, CHANGE);

  // GPS Not Working
  // ss.begin(GPSBaud);
}

//____________LOOP_______________________________________________________

void loop() {

  if (DEBUG) {
    getVSSSpeed();
  }

  // GPS
  //getGPSSpeed();
  //GPS Not working :(, use VSS only
  getVSSSpeed();
  // DEBUG
  printValues();

  // Speedo
  updateSpeedo();

}

//____________FUNCTIONS___________________________________________________

void getGPSSpeed() {

  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      // only use GPS speed when 5 Satelites are visible and minimum Speed is reached
      if (gps.speed.isValid() && gps.satellites.value() >= satAmount && gps.speed.kmph() >= minGPSSpeed) {

        // If GPS is available, use it
        speedKmH = gps.speed.kmph();
        Serial.println("GPS Speed set");

      } else {
        // If GPS is not available, fall back to vehicle speed sensor
        getVSSSpeed();
      }
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    //while (true);
    // Fall back to VSS if GPS device is not found
    getVSSSpeed();
  }

}



void getVSSSpeed() {

  // if new Pulst is available, calculate Speed in KmH
  if (newPulseDurationAvailable) {

    newPulseDurationAvailable = false;

    // filter out spikes
    if(inputDuration < vssIn[0]){
      return;
    }

    // interpolate VSS pulselength to speed in KmH
    speedKmH = multiMap(inputDuration, vssIn, vssOut, 20);

    //Serial.println("VSS Speed set");

  } else {
    // If no current speed signal is avaiable, activate interrupt to get new one
    attachInterrupt(digitalPinToInterrupt(inputPin), pulseLength, CHANGE);
  }

}

void updateSpeedo() {
  unsigned long currentMicros = micros();
  digitalWrite(analogOutPin, HIGH);
  // Map Speed to Speedometer Pulses
  outputValue = multiMap(speedKmH, speedoIn, speedoOut, 16);
  // Update the Speedo
  if (currentMicros - previousMicros_Speedo >= outputValue && outputValue <= minSpeed ) {
    // save the last time you blinked the LED
    previousMicros_Speedo = currentMicros;

    // Delay to recognise input
    delay(2);
    // Pull pin low to trigger speedo
    digitalWrite(analogOutPin, LOW);
    // Delay to limit Speedo to 140 KmH
    delay(7);
  }
  digitalWrite(analogOutPin, HIGH);
}

void pulseLength() {
  if (digitalRead(inputPin) == LOW)
  {
    if (!newPulse) {
      newPulse = true;
      startTime = micros(); //get time of pulse going down
    }
  }
  else
  {
    if (newPulse) {
      // detach interrupt until data has been handled
      detachInterrupt(digitalPinToInterrupt(inputPin));
      newPulse = false;
      stopTime = micros();  //get time of pulse going up
      inputDuration = stopTime - startTime;
      newPulseDurationAvailable = true;
    }
  }
}

void printValues() {

  unsigned long currentMicros = micros();

  // Houw often to scan the input (1 times per second)
  if (currentMicros - previousMicros_InputRead >= 1000000) {
    // save the last time you blinked the LED
    previousMicros_InputRead = currentMicros;

    //      print the results to the Serial Monitor:
    Serial.print("Satelites = ");
    Serial.print(gps.satellites.value());
    Serial.print("\t GPS Speed = ");
    Serial.print(gps.speed.kmph());
    Serial.print("\t VSS = ");
    Serial.print(inputDuration);
    Serial.print("\t VSS Speed = ");
    Serial.print(speedKmH);
    Serial.print("\t output = ");
    Serial.println(outputValue);

  }

}
