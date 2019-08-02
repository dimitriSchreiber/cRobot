//
// Created by bunny on 8/1/19.
//

#ifndef UNTITLED_FPGAENCODER_H
#define UNTITLED_FPGAENCODER_H
#include "fpgaAddressComm.h"

namespace CTRobot{
    class fpgaEncoder{
    public:
        fpgaEncoder(unsigned char* aEncoderAddr, unsigned char* aEncoderResetAddr, uint32_t aIndex): encoderAddr(aEncoderAddr), encoderResetAddr(aEncoderResetAddr), encoderIndex(aIndex){};
        ~fpgaEncoder(){};

        int32_t readValue(){
            return (int32_t)encoderAddr.readWord();
        }

        void reset(){
            encoderResetAddr.writeWord(0);
        }

        uint32_t getIndex(){
            return encoderIndex;
        }


    protected:
        fpgaCommunication encoderAddr;
        fpgaCommunication encoderResetAddr;
        uint32_t encoderIndex;
    };

}

#endif //UNTITLED_FPGAENCODER_H
