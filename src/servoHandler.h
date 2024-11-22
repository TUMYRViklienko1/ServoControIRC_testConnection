#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include <Arduino.h>
#include <Wire.h>                             
#include <iarduino_MultiServo.h>
#include <StandardCplusplus.h>
#include <vector>
#define MAX_DOF 3
#define MAX_SERVO 4
const int SERVO_MIN = 130;
const int SERVO_MAX = 470;
const int INITIAL_ANGLE = 90;
const int MAX_ANGLE = 180;
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
    void getForwardKinematic();
    double interpolateAngle(double theta_s, int theta_f, double tf, int servo_id);
    void setForwardKinematic(double duration, uint8_t pinMap[MAX_SERVO]);
private:
    std::vector<int> parseString(const String &dataFromSerialPort);

    servoData servoData;
    iarduino_MultiServo SercoController;
    uint16_t angleForwardKinematic[MAX_SERVO] = {90, 90, 90, 90}; // Servo angles: Shoulder, Elbow, Base + claw
    static uint16_t previousAngles[MAX_DOF];
    int counterForServo = 0; 
    std::vector<servoData> autoModeAngles;  
    static int timePerSleep;
};



#endif //SERVOHANDLER_H
