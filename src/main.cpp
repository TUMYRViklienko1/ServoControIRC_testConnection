#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include "../lib/servoHandler/src/servoHandler.h"



uint8_t baseServo = 0;
uint8_t shoulderServo = 1;
uint8_t elbowServo = 2;
uint8_t clawServo = 3 ;

servoHandler mServoController(baseServo,shoulderServo,elbowServo,clawServo);
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000); // Set timeout to 100 milliseconds
  while (!Serial)
  {}
  mServoController.servoControllerBegin();
}

void loop() {
  mServoController.setForwardKinematic(1);

}