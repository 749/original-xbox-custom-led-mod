#include <Arduino.h>
#include <Wire.h>

typedef enum {
  STATE_ERROR,
  STATE_CUSTOM
} CODE_STATE;

int pinGreenLed = A1;
int pinRedLed = A0;
int lastValueRed = 0;
int lastValueGreen = 0;
CODE_STATE status = STATE_ERROR; // 0 -> error passthrough; 1 -> error detect and custom colours

void onReceive(int data);
void onRequest();

void setup() {
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(pinGreenLed, INPUT_PULLUP);
  pinMode(pinRedLed, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  Wire.onReceive(&onReceive);
  Wire.onRequest(&onRequest);
  Serial.println("Ready!");
}

void loop() {
  bool green = digitalRead(pinGreenLed);
  bool red = digitalRead(pinRedLed);
  
  if(status != STATE_ERROR && red) {
    Serial.println("Error occured after Boot.");
    status = STATE_ERROR;
  }

  switch (status)
  {
  case STATE_CUSTOM:
    digitalWrite(9, 1);
    digitalWrite(10, 1);
    digitalWrite(11, 1);
    break;
  
  default:
  case STATE_ERROR:
    if(red != lastValueRed) {
      lastValueRed = red;
      Serial.print("red: ");
      Serial.println(red);
    }
    if(green != lastValueGreen) {
      lastValueGreen = green;
      Serial.print("green: ");
      Serial.println(green);
    }
    digitalWrite(9, red);
    digitalWrite(10, green);
    digitalWrite(11, !red && !green);
  }
}

void onReceive(int data) {
  Serial.print("Got a I2C-Data! ");
  Serial.println(data);

}

void onRequest() {
  Serial.println("Got a I2C-Request!");
}
