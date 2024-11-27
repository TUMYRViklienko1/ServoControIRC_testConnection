#include "servoHandler.h"

servoData::servoData(const std::vector<uint16_t> &dataFromSerialPort)
    : theta_1(dataFromSerialPort.at(0)), theta_2(dataFromSerialPort.at(1)), theta_3(dataFromSerialPort.at(2)), claw(dataFromSerialPort.at(3)) {}

 std::vector<uint16_t> servoData::servoDataGet() const{
  std::vector<uint16_t> temp =  {theta_1,theta_2,theta_3,claw};
  return temp;
}

servoHandler::servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
 uint8_t elbowServoPin, uint8_t clawServoPin){
    SercoController.servoSet(baseServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 0
    SercoController.servoSet(shoulderServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 1
    SercoController.servoSet(elbowServoPin, MAX_ANGLE, SERVO_MIN, SERVO_MAX); // Servo at pin 2
    SercoController.servoSet(clawServoPin,MAX_ANGLE, SERVO_MIN, SERVO_MAX);

    SercoController.begin(&Wire, 0x40, 1000);
}

std::vector<uint16_t> servoHandler::getFromSerialPort(){
  if (Serial.available())
  {
    const String dataFromSerialPort = Serial.readStringUntil('\n');
    std::vector<uint16_t> dataFromString = parseString(dataFromSerialPort);
    autoModeFlagCheck(dataFromString);
    return dataFromString;
  }
  return std::vector<uint16_t>();
}

bool servoHandler::autoModeFlagCheck(const std::vector<uint16_t> &dataFromString){
  switch (*dataFromString.begin())
  {
  case STARTAUTOMODE:
    {
    timePerSleep = dataFromString.at(1);
    numberOfElemetns = dataFromString.at(2);
    return true;
    }
  case STOPAUTOMODE:
    {
    timePerSleep = 0;
    numberOfElemetns = 0;
    return true;
    }
  default:
    return false;
  }
}

double servoHandler::interpolateAngle(const double theta_s,const int theta_f,const double tf,const int servo_id){

  static double endt[3];  // Static array to store end angles for each servo
  static unsigned long ts[3];  // Static array to store timestamps for each servo
  static double thet[3];  // Static array to store final angles for each servo

  // Check if this is a new target angle for the servo
  if (endt[servo_id] != theta_f) {
    ts[servo_id] = millis();  // Store the timestamp for this servo
    endt[servo_id] = theta_f; // Update the target angle for this servo
  }

  // Cubic polynomial coefficients
  const double a0 = theta_s;
  const double a1 = 0;
  const double a2 = (3.0 / pow(tf, 2)) * (theta_f - theta_s);
  const double a3 = -(2.0 / pow(tf, 3)) * (theta_f - theta_s);
  const double t = (double)(millis() - ts[servo_id]) / 1000;  // Time elapsed in seconds

  // Calculate the interpolated angle
  if (t <= tf) {
    thet[servo_id] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    thet[servo_id] = theta_f;  // Snap to final angle after duration
  }

  return thet[servo_id];
}

void servoHandler::setForwardKinematic(double duration, const servoData &servoAngles){
  // Initialize angles on first loop iteration
  if (counterForServo == 0) {
    for (unsigned int & previousAngle : previousAngles)
    {
      previousAngle = 90;
    }
  }

  std::vector<uint16_t> temp = servoAngles.servoDataGet();
  // Interpolate angles over the specified duration
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration * 1000); // Convert duration to milliseconds

  while (millis() < endTime) {
    for (int i = 0; i < MAX_DOF; i++)
    {
      SercoController.servoWrite(i,interpolateAngle(previousAngles[i],temp.at(i),duration,i));
    }
    delay(10); 
  }
  for (int i = 0; i < MAX_DOF; i++)
  {
    previousAngles[i] = temp.at(i);
  }

  counterForServo++;
}

std::vector<uint16_t> servoHandler::parseString(const String &dataFromSerialPort) const{
      
  int valueIndex = 0; // Index for angleForwardKinematic array
  int startIdx = 0; // Start index for substring
  int commaIdx;
  std::vector<uint16_t> dataFromString;
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

void servoHandler::serialPortHandler() {
  const std::vector<uint16_t> serialPortData = getFromSerialPort();

  if (serialPortData.empty())
    return;



  const servoData temp(serialPortData);
  if (numberOfElemetns != 0 && counter != 0)
  {
    autoModeAngles.push_back(temp);
    numberOfElemetns--;
  }
  else
    autoModeAngles.at(0) = temp;

  for (size_t i = 0; i < autoModeAngles.size(); ++i) {
    setForwardKinematic(timePerSleep, autoModeAngles.at(i));
  }
  counter++;
}
