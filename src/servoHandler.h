#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include <Arduino.h>
#include <Wire.h>                             
#include <iarduino_MultiServo.h>
#define MAX_DOF 3

struct servoData
{
    uint8_t theta_1;
    uint8_t theta_2;
    uint8_t theta_3;
    bool claw = 1;
    
    servoData(uint8_t theta_1,uint8_t theta_2,uint8_t theta_3, bool claw);
};

class servoHandler {
public:
    servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
    uint8_t elbowServoPin, uint8_t clawServoPin);
    void getForwardKinematic();
    double interpolateAngle(double theta_s, int theta_f, double tf, int servo_id);
private:
    servoData servoData;
    iarduino_MultiServo SercoController;
    int angleForwardKinematic[MAX_DOF + 1] = {90, 90, 90, 90}; // Servo angles: Shoulder, Elbow, Base + claw
};



#endif //SERVOHANDLER_H
