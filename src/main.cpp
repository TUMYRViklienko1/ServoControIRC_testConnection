#include <Arduino.h>
#include <Servo.h>
#define MAX_DOF 3
#define MAX_BUFFER 12

int servoPin_1 = 9;
int servoPin_2 = 10;
int servoPin_3 = 11;
Servo shoulderServo;
Servo elbowServo; // Corrected name
Servo baseServo;
int angleForwardKinematic[MAX_DOF] = {90,90,90}; // Corrected name
int switch1;
float switch1Smoothed;
float switch1Prev;
int counterForServo = 0;
 static double angPrevShoulder =0, angPrevElbow = 0;
void getForwardKinematic();
double forwardKinematicShoulder(double theta_s, int theta_f, double tf);
double forwardKinematicElbow(double theta_s, int theta_f, double tf);
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(0.1);
  shoulderServo.attach(servoPin_1, 440, 2400);
  elbowServo.attach(servoPin_2, 440, 2400);
}

void loop() {
 
  double angShoulder, angElbow, angBase;
  double duration =0.1; 
  getForwardKinematic();

    for (int i = 0; i < MAX_DOF; i++)
    {
      Serial.print(i);
      Serial.println(angleForwardKinematic[i]);
    }
    if (counterForServo == 0) {
      angPrevShoulder = 90;
      angPrevElbow = 90;
      angBase = 90;
    }

    // Interpolate angles over a duration
    unsigned long startTime = millis();
    unsigned long endTime = startTime + (duration * 1000); // Convert duration to milliseconds

    while (millis() < endTime) {
      double t = (millis() - startTime) / 1000.0; // Time elapsed in seconds
      angBase = forwardKinematicElbow(angPrevElbow, angleForwardKinematic[2], duration);
      angShoulder = forwardKinematicShoulder(angPrevShoulder, angleForwardKinematic[0], duration);
      angElbow = forwardKinematicElbow(angPrevElbow, angleForwardKinematic[1], duration);
      baseServo.write(angBase);
      shoulderServo.write(angShoulder);
      elbowServo.write(angElbow);

      delay(10); 
    }

    Serial.println("Successfully moved!");
    angPrevShoulder = angleForwardKinematic[0];
    angPrevElbow = angleForwardKinematic[1];
    counterForServo++;

}

void getForwardKinematic() {
  while (Serial.available() > 0) {
    char modificator = Serial.read();
    switch (modificator)
    {
    case 's':
      angleForwardKinematic[0] = Serial.parseInt();
      break;
    case 'e':
      angleForwardKinematic[1] = Serial.parseInt();
      break;
    case 'b':
      angleForwardKinematic[2] = Serial.parseInt();
      break;
    case 'r':
      for (int i = 0; i < MAX_DOF; i++)
      angleForwardKinematic[i] = 90;
      break;
    default:
      break;
    }


    // Clear the remaining characters in the buffer
    while (Serial.available() > 0) {
      int trash = Serial.read();
    }
  }
}

double forwardKinematicShoulder(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // final angle

  if (endt != theta_f) {
    ts = millis();
    endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s); // Fixed division
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s); // Fixed division
  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  }
  return thet;
}
double forwardKinematicElbow(double theta_s, int theta_f, double tf) {
  double a0, a1, a2, a3;
  double t;
  static double endt;
  static unsigned long ts;
  static double thet; // final angle

  if (endt != theta_f) {
    ts = millis();
    endt = theta_f;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s); // Fixed division
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s); // Fixed division
  t = (double)(millis() - ts) / 1000;

  if (t <= tf) {
    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  }
  return thet;
}