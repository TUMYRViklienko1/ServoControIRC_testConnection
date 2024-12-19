#include "servoHandler.h"
#define DURATION 1

servoData::servoData(int dataFromSerialPort[4])
    : theta_1(dataFromSerialPort[0]), theta_2(dataFromSerialPort[1]), theta_3(dataFromSerialPort[2]), claw(dataFromSerialPort[3]) {}

 std::vector<uint16_t> servoData::servoDataGet() const{
  std::vector<uint16_t> temp =  {theta_1,theta_2,theta_3,claw};
  return temp;
}

servoHandler::servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin, uint8_t elbowServoPin,
uint8_t clawServoPin): baseServo(baseServoPin), shoulderServo(shoulderServoPin),elbowServo(elbowServoPin),
clawServo(clawServoPin) {}

void servoHandler::servoControllerBegin() {
  SercoController.servoSet(baseServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 0
  SercoController.servoSet(shoulderServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 1
  SercoController.servoSet(elbowServo, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 2
  SercoController.servoSet(clawServo,MAX_ANGLE, SERVO_MIN, SERVO_MAX);

  SercoController.begin();
}

void servoHandler::setForwardKinematic(const double duration){
  // Initialize angles on first loop iteration
  getFromSerialPort(); // Get the angles from serial input

  if (angleForwardKinematic[0] == -1) {
    durationPerStep = angleForwardKinematic[1];
    numberOfElements = angleForwardKinematic[2];
    return;
  }

  if (numberOfElements != 0) {
    const servoData temp(angleForwardKinematic);
    autoModeAngles.push_back(temp);
    numberOfElements--;
    if (numberOfElements == 0) {
      autoModeHandler();
    }
    return;
  }
  // Initialize angles on first loop iteration
  sendToServo(angleForwardKinematic[0],angleForwardKinematic[1],angleForwardKinematic[2]);
}


void servoHandler::getFromSerialPort() {
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

double servoHandler::forwardKinematicBase(double theta_s, int theta_f, double tf) {
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

double servoHandler::forwardKinematicShoulder(double theta_s, int theta_f, double tf) {
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

double servoHandler::forwardKinematicElbow(double theta_s, int theta_f, double tf) {
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

void servoHandler::autoModeHandler() {
  while (true)
  for (auto & autoModeAngle : autoModeAngles) {
    sendToServo(autoModeAngle.theta_1,autoModeAngle.theta_2,autoModeAngle.theta_3);
  }
}

void servoHandler::sendToServo(int theta_1, int theta_2, int theta_3) {
  if (counterForServo == 0) {
    for (double & previousAngle : previousAngles) {
      previousAngle = 90;
    }
  }

  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (DURATION * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {
    SercoController.servoWrite(baseServo,forwardKinematicBase(previousAngles[0], theta_1, DURATION));
    SercoController.servoWrite(shoulderServo,forwardKinematicShoulder(previousAngles[1], theta_2, DURATION));
    SercoController.servoWrite(elbowServo,forwardKinematicElbow(previousAngles[2], theta_3, DURATION));


    delay(10);
  }

  // Store previous angles for next loop iteration
  for (int i = 0; i < MAX_DOF; ++i) {
    previousAngles[i] = angleForwardKinematic[i];
  }

  counterForServo++;
}




