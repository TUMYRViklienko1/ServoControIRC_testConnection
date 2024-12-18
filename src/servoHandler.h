#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include <Arduino.h>
#include <Wire.h>                             
#include <iarduino_MultiServo.h>
#include <StandardCplusplus.h>
#include <vector>
#define MAX_DOF 3
#define MAX_SERVO 4
//[flag,timer,size] - start
//[flag] - stop
//[theta_1,theta_2,theta_3,theta_4] 

const uint16_t SERVO_MIN = 130;
const uint16_t SERVO_MAX = 470;
const uint16_t INITIAL_ANGLE = 90;
const uint16_t MAX_ANGLE = 180;
const uint8_t STARTAUTOMODE = -1;
const uint8_t STOPAUTOMODE = -2;
struct servoData
{
    uint16_t theta_1;
    uint16_t theta_2;
    uint16_t theta_3;
    bool claw = 1;
    
    servoData(uint16_t theta_1,uint16_t theta_2,uint16_t theta_3, bool claw);
};

class servoHandler {
public:
    servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
    uint8_t elbowServoPin, uint8_t clawServoPin);
    void getFromSerialPort();
    double interpolateAngle(double theta_s, int theta_f, double tf, int servo_id);
    void setForwardKinematic(double duration, uint8_t pinMap[MAX_SERVO]);
private:
    std::vector<int> parseString(const String &dataFromSerialPort);
    bool autoModeFlagCheck(const std::vector<int> &dataFromString);
    servoData mServoData;
    iarduino_MultiServo SercoController;
    uint16_t angleForwardKinematic[MAX_SERVO] = {90, 90, 90, 90}; // Servo angles: Shoulder, Elbow, Base + claw
    static uint16_t previousAngles[MAX_DOF];
    int counterForServo = 0; 
    std::vector<servoData> autoModeAngles;  
    int timePerSleep;
    static int numberOfElemetns;
};



#endif //SERVOHANDLER_H
