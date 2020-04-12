#ifndef WIREDCOMMS_H_
#define WIREDCOMMS_H_

#include "commsInterface.h"

class Wired: public commsInterface 
{
    public:
        uint8_t wirePin;
        int32_t readValue = 0; 
        
        Wired(uint8_t pin)
        {
            wirePin = pin;
        }

        void init(void);
        int32_t receive(void);
};

#endif