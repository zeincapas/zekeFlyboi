#ifndef MOTOR_H_
#define MOTOR_H_

#include "motorInterface.h"

class BLDC: public motorInterface 
{
    public:
        uint8_t motorPin;

        BLDC(uint8_t pin) 
        {
            motorPin = pin;
        }

        void init(void);
        void setSpeed(int32_t speed);
};

#endif