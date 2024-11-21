#include "servoHandler.h"

servoData::servoData(uint16_t theta_1,uint16_t theta_2,uint16_t theta_3, bool claw)
    : theta_1(theta_1), theta_2(theta_2), theta_3(theta_3), claw(claw) {}

servoHandler::servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
 uint8_t elbowServoPin, uint8_t clawServoPin){
    SercoController.servoSet(baseServoPin, 180, 130, 470); // Servo at pin 0
    SercoController.servoSet(shoulderServoPin, 180, 130, 470); // Servo at pin 1
    SercoController.servoSet(elbowServoPin, 180, 130, 470); // Servo at pin 2
    SercoController.servoSet(clawServoPin,180, 130, 470);

    SercoController.begin(&Wire, 0x40, 1000);
    
}

void servoHandler::getForwardKinematic(){
  if (Serial.available())
  {
    String dataFromSerialPort = Serial.readStringUntil('\n'); 
    int valueIndex = 0; // Index for angleForwardKinematic array
    int startIdx = 0; // Start index for substring
    int commaIdx;

    // Loop to parse the string
    while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1 && valueIndex < MAX_DOF) {
      // Extract the substring and convert to integer
      angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1; // Move to the next character after the comma
      valueIndex++;
    }

    // Handle the last value (after the final comma)
    if (valueIndex < MAX_DOF) {
      angleForwardKinematic[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();
    }
  }
}

double servoHandler::interpolateAngle(double theta_s, int theta_f, double tf, int servo_id){
  double a0, a1, a2, a3;
  double t;
  static double endt[3];  // Static array to store end angles for each servo
  static unsigned long ts[3];  // Static array to store timestamps for each servo
  static double thet[3];  // Static array to store final angles for each servo

  // Check if this is a new target angle for the servo
  if (endt[servo_id] != theta_f) {
    ts[servo_id] = millis();  // Store the timestamp for this servo
    endt[servo_id] = theta_f; // Update the target angle for this servo
  }

  // Cubic polynomial coefficients
  a0 = theta_s;
  a1 = 0;
  a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);

  t = (double)(millis() - ts[servo_id]) / 1000;  // Time elapsed in seconds

  // Calculate the interpolated angle
  if (t <= tf) {
    thet[servo_id] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    thet[servo_id] = theta_f;  // Snap to final angle after duration
  }

  return thet[servo_id];
}

void servoHandler::setForwardKinematic(double duration, uint8_t pinMap[MAX_SERVO]){
  double angShoulder, angElbow, angBase;

  getForwardKinematic(); // Get the angles from serial input

  // Initialize angles on first loop iteration
  if (counterForServo == 0) {
    for (int i = 0; i < MAX_DOF; i++)
    {
      previousAngles[i] = 90;
    }
  }

  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {
    for (int i = 0; i < MAX_DOF; i++)
    {
      SercoController.servoWrite(pinMap[i],interpolateAngle(previousAngles[i],angleForwardKinematic[i],duration,i));
    }
    delay(10); 
  }
  for (int i = 0; i < MAX_DOF; i++)
  {
    previousAngles[i] = angleForwardKinematic[i];
  }

  counterForServo++;
}