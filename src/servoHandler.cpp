#include "servoHandler.h"

servoData::servoData(uint16_t theta_1,uint16_t theta_2,uint16_t theta_3, bool claw)
    : theta_1(theta_1), theta_2(theta_2), theta_3(theta_3), claw(claw) {}

servoHandler::servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
 uint8_t elbowServoPin, uint8_t clawServoPin){
    SercoController.servoSet(baseServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 0
    SercoController.servoSet(shoulderServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 1
    SercoController.servoSet(elbowServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 2
    SercoController.servoSet(clawServoPin,MAX_ANGLE, SERVO_MIN, SERVO_MAX);

    SercoController.begin(&Wire, 0x40, 1000);
    
}

void servoHandler::getFromSerialPort(){
  if (Serial.available())
  {
    String dataFromSerialPort = Serial.readStringUntil('\n'); 
    std::vector<int> dataFromString = parseString(dataFromSerialPort);

    autoModeFlagCheck(*dataFromString.begin());
    if (*dataFromString.begin() ==  -1)
    {
      dataFromString[1] = timePerSleep;
      dataFromString[2] = numberOfElemetns;
    }
    else
    {
      for (int i = 0; i < MAX_SERVO; i++)
      {
             angleForwardKinematic[i] = dataFromString[i];
      }      
    }
}
}

bool servoHandler::autoModeFlagCheck(const std::vector<int> &dataFromString){
  switch (*dataFromString.begin())
  {
  case STARTAUTOMODE:
    {
    timePerSleep = dataFromString.at(1);
    numberOfElemetns = dataFromString.at(2);
    break;
    }
  case STOPAUTOMODE:
    {
//stop auto mode
    break;
    }
  default:
    break;
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

  getFromSerialPort(); // Get the angles from serial input
  
  
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

std::vector<int> servoHandler::parseString(const String &dataFromSerialPort){
      
  int valueIndex = 0; // Index for angleForwardKinematic array
  int startIdx = 0; // Start index for substring
  int commaIdx;
  std::vector<int> dataFromString;
      while ((commaIdx = dataFromSerialPort.indexOf(',', startIdx)) != -1) {
      // Extract the substring and convert to integer
      dataFromString[valueIndex] = dataFromSerialPort.substring(startIdx, commaIdx).toInt();
      startIdx = commaIdx + 1; // Move to the next character after the comma
      valueIndex++;
    }

    // Handle the last value (after the final comma)
    if (valueIndex < MAX_DOF) {
      dataFromString[valueIndex] = dataFromSerialPort.substring(startIdx).toInt();
    }

    return dataFromString;
}