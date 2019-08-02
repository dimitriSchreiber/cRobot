//
// Created by bunny on 7/31/19.
//

#ifndef UNTITLED_MOTORCONTROLLER_H
#define UNTITLED_MOTORCONTROLLER_H

extern "C"{
#include <rc/math/filter.h>
}

#include <map>
#include <cmath>
#include "fpgaAddressComm.h"
#include "fpgaEncoder.h"

namespace CTRobot{
class motorController{
public:
    motorController(unsigned char* PWMAddress, unsigned char* dirAddress, uint32_t anIndex):
            motorPWM(PWMAddress), motorDirection(dirAddress),
            motorIndex(anIndex), motorEnable(NULL), motorEncoder(nullptr){

        PIDfilter = rc_filter_empty();
        motorPWM.writeWord(0);
    };

    ~motorController(){
        delete(motorEncoder);
    }

    bool move(uint32_t aSpeed, keywords aKeyword);
    void stop();
    motorController& attachEncoder(unsigned char* aEncoderAddr, unsigned char* encoderResetAddr, uint32_t anIndex);
    int32_t readEncoder();
    void resetEncoder();
    uint32_t getIndex();
    motorController& setPIDValue(float aP = 0.005, float aI = 0.0005, float aD = 0.000006, float dt = 0.001);
    void runPID(int32_t aDestination);

protected:
    fpgaCommunication motorPWM;
    fpgaCommunication motorDirection;
    fpgaCommunication motorEnable;
    fpgaEncoder *motorEncoder;
    uint32_t motorIndex;
    rc_filter_t PIDfilter;
    };
}

#endif //UNTITLED_MOTORCONTROLLER_H
