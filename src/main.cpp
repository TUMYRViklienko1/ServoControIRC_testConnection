#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <iarduino_MultiServo.h>
#include <SPI.h>
#include "servoHandler.h"
#include "../lib/servoHandler/src/servoHandler.h"

#define MAX_DOF 3
#define MAX_BUFFER 12

iarduino_MultiServo SercoController;

uint8_t baseServo = 0;
uint8_t shoulderServo = 1;
uint8_t elbowServo = 2;
uint8_t clawServo = 3 ;


int angleForwardKinematic[MAX_DOF + 1] = {90, 90, 90, 90}; // Servo angles: Shoulder, Elbow, Base + claw
int counterForServo = 0;

double angPrevShoulder = 0, angPrevElbow = 0, angPrevBase = 0;

double previousAngles[MAX_DOF] = {0,0,0};

void getFromSerialPort();
double forwardKinematicBase(double theta_s, int theta_f, double tf);
double forwardKinematicShoulder(double theta_s, int theta_f, double tf);
double forwardKinematicElbow(double theta_s, int theta_f, double tf);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000); // Set timeout to 100 milliseconds
  while (!Serial)
  {}
  SercoController.servoSet(baseServo, 180, 130, 470); // Servo at pin 0
  SercoController.servoSet(shoulderServo, 180, 130, 470); // Servo at pin 1
  SercoController.servoSet(elbowServo, 180, 130, 470); // Servo at pin 2
  SercoController.servoSet(clawServo,180, 130, 470);
  //SercoController.servoSet(SERVO_ALL, SERVO_MG90);
  SercoController.begin();
}

void loop() {
  double angShoulder, angElbow, angBase;
  double duration = 1;  // Duration in seconds

  getFromSerialPort(); // Get the angles from serial input

  // Initialize angles on first loop iteration
  if (counterForServo == 0) {
    for (double & previousAngle : previousAngles) {
      previousAngle = 90;
    }
  }

  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {

     SercoController.servoWrite(baseServo,forwardKinematicBase(previousAngles[0], angleForwardKinematic[0], duration));
     SercoController.servoWrite(shoulderServo,forwardKinematicShoulder(previousAngles[1], angleForwardKinematic[1], duration));
     SercoController.servoWrite(elbowServo,forwardKinematicElbow(previousAngles[2], angleForwardKinematic[2], duration));


    delay(10);
  }

  // Store previous angles for next loop iteration
  for (int i = 0; i < MAX_DOF; ++i) {
    previousAngles[i] = angleForwardKinematic[i];
  }

  counterForServo++;
}

// Function to read angles from serial input
void getFromSerialPort() {
  if (Serial.available())
  {
    String dataFromSerialPort = Serial.readStringUntil('\n'); 
    int valueIndex = 0; // Index for angleForwardKinematic array
    int startIdx = 0; // Start index for substring
    int commaIdx;

    // Loop to parse the string
    while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1) {
      // Extract the substring and convert to integer
      angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1; // Move to the next character after the comma
      valueIndex++;
    }

    // Handle the last value (after the final comma)
    angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();

  }
  
}

// Function to compute shoulder angle interpolation over time (cubic polynomial)
double forwardKinematicShoulder(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

  if (endt != theta_f) {
    ts = millis();
    endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}

// Function to compute elbow angle interpolation over time (cubic polynomial)
double forwardKinematicElbow(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

  if (endt != theta_f) {
    ts = millis();
    endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}

double forwardKinematicBase(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // Final angle

  if (endt != theta_f) {
    ts = millis();
    endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    thet = theta_f;  // Snap to final angle after duration
  }

  return thet;
}






