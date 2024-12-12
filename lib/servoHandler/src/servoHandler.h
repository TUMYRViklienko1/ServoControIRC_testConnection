#ifndef SERVOHANDLER_H
#define SERVOHANDLER_H
#include <Arduino.h>
#include <Wire.h>

    // return dataFromString;
#include <iarduino_MultiServo.h>
#include <StandardCplusplus.h>
#include <vector>
#define MAX_DOF 3
#define MAX_SERVO 4
//[flag,timer,size] - start
//[flag] - stop
//[theta_1,theta_2,theta_3,theta_4] 
constexpr uint16_t SERVO_MIN = 130;
constexpr uint16_t SERVO_MAX = 470;
constexpr uint16_t INITIAL_ANGLE = 90;
constexpr uint16_t MAX_ANGLE = 180;
constexpr uint8_t STARTAUTOMODE = -1;
constexpr uint8_t STOPAUTOMODE = -2;
struct servoData
{
    uint16_t theta_1;
    uint16_t theta_2;
    uint16_t theta_3;
    bool claw = true;
    
    explicit servoData(const std::vector<uint16_t> &dataFromSerialPort);
    servoData();
    std::vector<uint16_t> servoDataGet() const;
};

class servoHandler {
public:
    servoHandler(uint8_t baseServoPin, uint8_t shoulderServoPin,
    uint8_t elbowServoPin, uint8_t clawServoPin);
    void serialPortHandler();
private:
    void setForwardKinematic(double duration, const servoData &servoAngles);
    static double interpolateAngle(double theta_s, int theta_f, double tf, int servo_id);
    std::vector<uint16_t> parseString(const String &dataFromSerialPort) const;
    bool autoModeFlagCheck(const std::vector<uint16_t> &dataFromString);
    std::vector<uint16_t> getFromSerialPort();

    servoData mServoData;
    iarduino_MultiServo SercoController;
    static uint16_t previousAngles[MAX_DOF];
    int counterForServo = 0; 
    std::vector<servoData> autoModeAngles;  
    int timePerSleep{};
    static int numberOfElemetns;
    static int counter;
};



#endif //SERVOHANDLER_H
