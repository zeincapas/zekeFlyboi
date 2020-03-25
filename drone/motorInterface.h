#ifndef MOTORINTERFACE_H_
#define MOTORINTERFACE_H_

class motorInterface {
    public:
        virtual void init() = 0;
        virtual void setSpeed() = 0;
};

#endif