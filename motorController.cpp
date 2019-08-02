//
// Created by bunny on 7/31/19.
//
#include "motorController.h"

namespace CTRobot{
    bool motorController::move(uint32_t aSpeed, keywords aKeyword) {
//      acquire current state and compute altered state
        uint32_t temp = motorDirection.readWord();
        temp = temp & ~(1UL<<motorIndex);
        temp = temp | (controlKeywords[aKeyword]<<motorIndex);

        motorDirection.writeWord(temp);
        motorPWM.writeWord(aSpeed);

        return true;
    }

    void motorController::stop() {
        motorPWM.writeWord((uint32_t)0);
    }

    motorController &motorController::attachEncoder(unsigned char *aEncoderAddr, unsigned char* encoderResetAddr, uint32_t anIndex) {
        if(motorEncoder != nullptr)
            delete(motorEncoder);
        motorEncoder = new fpgaEncoder(aEncoderAddr, encoderResetAddr, anIndex);
        resetEncoder();
        return *this;
    }

    int32_t motorController::readEncoder() {
        if(motorEncoder == nullptr){
            throw "Encoder not attached";
        }
        return motorEncoder->readValue();
    }

    void motorController::resetEncoder() {
        if(motorEncoder == nullptr){
            throw "Encoder not attached";
        }
        motorEncoder->reset();
    }

    uint32_t motorController::getIndex() {
        return motorIndex;
    }

    motorController& motorController::setPIDValue(float aP, float aI, float aD, float dt) {
        rc_filter_pid(&PIDfilter,aP,aI,aD,1*dt,dt);
        return *this;
    }

    void motorController::runPID(int32_t aDestination) {
        if(motorEncoder == nullptr){
            throw "Encoder not attached";
        }
        //ask Dmitri about scaling
        int32_t PWMOutput = rc_filter_march(&PIDfilter,aDestination - readEncoder())*20;
        auto motorDir = (PWMOutput >= 0) ? (CTRobot::keywords::forward) : ( CTRobot::keywords::backward);
        PWMOutput = fabs(PWMOutput);

        PWMOutput = (PWMOutput >= 2047) ? 2047 : PWMOutput;

        move(PWMOutput, motorDir);
    }


}
