#ifndef MOTORINTERFACE_H_
#define MOTORINTERFACE_H_

class motorInterface 
{
    public:
        virtual void init(void) = 0;
        virtual void setSpeed(int32_t) = 0;
};

#endif