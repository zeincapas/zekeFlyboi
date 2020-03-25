#ifndef COMMSINTERFACE_H_
#define COMMSINTERFACE_H_


class commsInterface {
    public:
        virtual void init() = 0;
        virtual int32_t receive() = 0;
};

#endif 