// ---------------------------------------------------------
// Code for the Close Pass Detector V4.
// Created by Cyclist Tom to help save lives.
// Ride safe.
// Version 4.3, dated 2018-03-25.
// ---------------------------------------------------------

#include "SevSeg.h"

#define MAX_DISTANCE 150 // Max distance in cm (for sensor only, not display)
#define LED_CM_LIMIT 150 // Max distance in cm (for display, not sensor)
//#define DELAY 5 //delay between sensor polling in ms
#define MIN_CM_RELIABLE 16 //minimum reliable sensor distance (i.e. 6" as per datasheet)
#define NUM_MEAS 5 //number of measurements averaged
#define NUM_MEAS_REQ 3 //number of reasonable measurements required
#define HUGE_DIST 9999 //huge distance
 
SevSeg sevseg; //Instantiate a seven segment controller object

unsigned long pingTimer; // timer used to control how often sensor is used
unsigned long PING_INTERVAL = 0; //only needs to be non-zero if sensor can't handle rapid polling
unsigned long HUGE_DELAY = 100000; //~10 times LED_HOLD_INTERVAL
unsigned int cmUS; // Store ultrasonic ping distances

unsigned long LEDtimer; //timer used to hold value on LED for LED_HOLD_INTERVAL ms
unsigned long LED_HOLD_INTERVAL = 10000; //in milliseconds
unsigned int cm_held; // cm value held on LED
float AN2CM; //analog to cm conversion

const int anPin = 2;

void setup() {
  // setup is only run once:
  Serial.begin(115200); //if using Serial Monitor, make sure its 'baud' number matches this!
  
  pingTimer = millis() + 1000; // First ping start in ms.

  cmUS = HUGE_DIST;
  cm_held = HUGE_DIST;
  LEDtimer = 0;
  /*According to LV-MaxSonar-EZ datasheet: A 5V supply yields ~9.8mV/in
    And analogRead(): 5 volts / 1024 units or, .0049 volts (4.9 mV) per unit.
    AN2CM [cm/unit] = [cm/inch]*[mV/unit]/[mV/inch]*/
  AN2CM = 2.54 * 4.9 / 9.8;

  byte numDigits = 3;
  byte digitPins[] = {4,7,6}; //V2
  byte segmentPins[] = {9,5,2,12,11,8,3,13}; //a, b, c, d, e, f, g, D.P. //V2
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_CATHODE; // See README.md for options
  bool updateWithDelays = false; // Default. Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros);
  sevseg.setBrightness(100); //ledOnTime is set here as map(brightness, 0, 100, 1, 2000) where 2000 is in microseconds.
}

void loop() { 

  //each loop takes ~2ms (excluding DELAY)

  if (millis() >= pingTimer) { 
      /*pingTimer is only needed for a sensor that can't handle rapid polling*/
      /*time for a new ultrasonic measurement*/
      pingTimer = millis() + PING_INTERVAL;
      //cmUS = HUGE_DIST;
      cmUS = getDistance();
      //cm = HUGE_DIST;
      oneSensorCycle(); // this might update cm_held & LEDtimer
  }
  if (LEDtimer > 0 & millis() >= LEDtimer) {
      /*LEDtimer was previously set but time has elapsed so reset variables*/
      LEDtimer = 0; //set to 0 meaning no timer
      cm_held = HUGE_DIST; //might not be needed
  }
  if (LEDtimer > 0 & ((millis() + HUGE_DELAY) < LEDtimer)) {
      /*millis() might've wrapped back to 0 (can happen every ~50 days)*/
      LEDtimer = millis() + LED_HOLD_INTERVAL;
  }
  if (pingTimer > 0 & ((millis() + HUGE_DELAY) < pingTimer)) {
      /*millis() might've wrapped back to 0 (can happen every ~50 days)*/
      pingTimer = millis() + PING_INTERVAL;
  }

  //Serial.print("loop() Time: ");
  //Serial.print(millis());
  //Serial.println("ms");

  sevseg.refreshDisplay(); // Must run repeatedly. This refresh is not enough if DELAY>0.
  
}

float getDistance() {

    int numReasMeas = 0; //this will be incremented with every reasonable measurement
    float cm, cm_avg;
    float cm_sum = 0;
    //float inch, inch_avg
    //float inch_sum = 0;

    for (int i = 0; i < NUM_MEAS ; i++)
    {
        //inch = analogRead(anPin) / 2.0 + (6.0/2.54);
        cm = analogRead(anPin) * AN2CM;
        //Serial.print("cm: ");
        //Serial.println(cm);
        //inch_sum += inch;
        if (cm > 0 && cm < MAX_DISTANCE)
        {
            cm_sum += cm;
            numReasMeas++;
        }
        

        //delayAndRefresh(DELAY);
    }

    if (numReasMeas >= NUM_MEAS_REQ)
    {
        /*there were enough good measurements to average*/
        cm_avg = cm_sum / numReasMeas;
        //inch_avg = inch_sum / NUM_MEAS;
    }
    else
    {
        /*there were not enough good measurements, so return 9999*/
        cm_avg = HUGE_DIST;
    }
    
    return cm_avg;
}

/*
void delayAndRefresh(int myDelay) {
  for (int i = 0; i < myDelay; i++)
  {
    delay(1); //delay in ms
    sevseg.refreshDisplay();
  }
}*/

void oneSensorCycle() { // Do something with the results.
  //Serial.print("US Sensor: ");
  //Serial.println(cmUS);
  //Serial.println("cm");
  //Serial.print("Time: ");
  //Serial.print(millis());
  //Serial.println("ms");
  //sevseg.refreshDisplay();
  //sanity check the measured distances
  unsigned int cm = 0; // Store good ping distance within this oneSensorCycle().
  if ((cmUS > 0) & (cmUS < MAX_DISTANCE)) {
      cm = cmUS;
  }
  if (LEDtimer == 0) { //LEDtimer = 0 so no value is held on display
    if (cm == 0) { //sensor is not returning anything and no value is being held on display
      //Serial.println("setNumber to 9999 since cm==0");
      sevseg.setNumber(HUGE_DIST, 0); //set LED to all blank or dashes
    }
    else if (cm > 0 & cm <= LED_CM_LIMIT) { //sensor value is good and no value is being held on display
      //Serial.print("setNumber to cm=");
      //Serial.println(cm);
      sevseg.setNumber(cm, 0); //display close pass distance in cm
      cm_held = cm;
      LEDtimer = millis() + LED_HOLD_INTERVAL;
    }
    else if (cm >= LED_CM_LIMIT) {
      //Serial.println("setNumber to 9999 since cm >= LED_CM_LIMIT");
      sevseg.setNumber(HUGE_DIST, 0); //pass not close enough
    }
    else { //cm < 0 or less than 18cm (due to loss of sensor signal)
      /*Negative cm should never happen, but just in case it does LED display will show dashes for invalid number*/
      //Serial.println("setNumber to 9999 since cm < 0");
      sevseg.setNumber(HUGE_DIST, 0);
    }
  }
  else { //LEDtimer non-zero so previous close pass is being held on display
    // code below updates cm displayed if new one is closer.
    if (cm < cm_held && cm > 0) { //current close pass was closer than the one that first triggered LED display
      //Serial.print("setNumber to updated cm=");
      //Serial.println(cm);
      sevseg.setNumber(cm, 0); //update close pass distance (will be held for LED_HOLD_INTERVAL)
      LEDtimer = millis() + LED_HOLD_INTERVAL;
      cm_held = cm;
    }
    else if (cm == 0) { //sensor now not returning anything
      //Serial.print("cm == 0 so setNumber to cm_held=");
      //Serial.println(cm_held);
      sevseg.setNumber(cm_held, 0); //keep displaying previous close pass distance (will be held for rest of LED_HOLD_INTERVAL) 
    }
    else {
      //Serial.print("default so setNumber to cm_held=");
      //Serial.println(cm_held);
      sevseg.setNumber(cm_held, 0); //default to holding value on display
    }
  }
}


