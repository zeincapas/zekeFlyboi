#ifndef DRONELOGIC_H_
#define DRONELOGIC_H_

struct Yaw
{
    double curr;
    double out;
    double set;
    double Kp;
    double Ki;
    double Kd;            
};

struct Roll 
{
    double curr;
    double out;
    double set;
    double Kp;
    double Ki;
    double Kd;
};

struct Pitch
{
    double curr;
    double out;
    double set;
    double Kp;
    double Ki;
    double Kd;
};

class Drone 
{
    private:
        double rawYaw, rawPitch, rawRoll;

    public:
        // void yawMOtorSet(ctrlOutYaw);
        // void pitchMotorSet(double*);
        // void rollMotorSet(double*);
        void complementaryFilter(void);
        void readSensorVal(void);
        // void thrustMotorSet(ctrlOutThrust);

        void init(void);
        void printData(void);
        // void compute(void);

        // TRANSLATE MOTOR THRUST COMBINATIONS TO THRUST, ROLL, PITCH, DRONE
        
};

#endif