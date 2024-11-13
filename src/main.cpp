#include <Arduino.h>
#include <Servo.h>

#define MAX_DOF 3
#define MAX_BUFFER 12

int servoPin_1 = 9;
int servoPin_2 = 10;
int servoPin_3 = 11;

Servo shoulderServo;
Servo elbowServo;
Servo baseServo;

int angleForwardKinematic[MAX_DOF] = {90, 90, 90}; // Servo angles: Shoulder, Elbow, Base
int counterForServo = 0;

static double angPrevShoulder = 0, angPrevElbow = 0, angPrevBase = 0;

void getForwardKinematic();
double forwardKinematicShoulder(double theta_s, int theta_f, double tf);
double forwardKinematicElbow(double theta_s, int theta_f, double tf);
double forwardKinematicBase(double theta_s, int theta_f, double tf);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000); // Set timeout to 100 milliseconds
  
  baseServo.attach(servoPin_1, 440, 2400);
  shoulderServo.attach(servoPin_2, 440, 2400); // Attach servo with min and max pulse
  elbowServo.attach(servoPin_3, 440, 2400);
}

void loop() {
  double angShoulder, angElbow, angBase;
  double duration = 1;  // Duration in seconds

  getForwardKinematic(); // Get the angles from serial input

  for (int i = 0; i < MAX_DOF; i++) {
    Serial.print(i);
    Serial.println(angleForwardKinematic[i]);
  }

  // Initialize angles on first loop iteration
  if (counterForServo == 0) {
    angPrevBase = 90;
    angPrevShoulder = 90;
    angPrevElbow = 90;
  }

  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {
    angBase = forwardKinematicBase(angPrevBase, angleForwardKinematic[0], duration);  // Direct assignment for base angle
    angShoulder = forwardKinematicShoulder(angPrevShoulder, angleForwardKinematic[1], duration);
    angElbow = forwardKinematicElbow(angPrevElbow, angleForwardKinematic[2], duration);
if (angBase >= 180 || angShoulder >= 180 || angElbow >= 180)
{
    return;
}
else {
    baseServo.write(angBase);    // Move base servo
    shoulderServo.write(angShoulder);  // Move shoulder servo
    elbowServo.write(angElbow);  // Move elbow servo
}

  
    delay(10); // Short delay to smooth movement
  }

  Serial.println("Successfully moved!");

  // Store previous angles for next loop iteration
  angPrevBase = angleForwardKinematic[0];
  angPrevElbow = angleForwardKinematic[1];
  angShoulder = angleForwardKinematic[2];

  counterForServo++;
}

// Function to read angles from serial input
void getForwardKinematic() {
  while (Serial.available() > 0) {
    char modificator = Serial.read();
    switch (modificator) {
      case 'b':
        Serial.println(angleForwardKinematic[2]);
        angleForwardKinematic[0] = Serial.parseInt(); // Set base angle
        break;
      case 's':
        Serial.println(angleForwardKinematic[0]);
        angleForwardKinematic[1] = Serial.parseInt(); // Set shoulder angle
        break;
      case 'e':
        Serial.println(angleForwardKinematic[1]);
        angleForwardKinematic[2] = Serial.parseInt(); // Set elbow angle
        break;
      case 'r':
        for (int i = 0; i < MAX_DOF; i++)  // Reset all angles to 90
          angleForwardKinematic[i] = 90;
        break;
      default:
        break;
    }

    // Clear any remaining characters in the buffer
    while (Serial.available() > 0) {
      int trash = Serial.read();
    }
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




