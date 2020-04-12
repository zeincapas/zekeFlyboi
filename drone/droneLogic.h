#ifndef DRONELOGIC_H_
#define DRONELOGIC_H_

class Drone 
{
    private:
        struct roll 
        {
            double curr;
            double out;
            double set;
            double Kp;
            double Ki;
            double Kd;
        };

        struct pitch
        {
            double curr;
            double out;
            double set;
            double Kp;
            double Ki;
            double Kd;
        };




    public:
        // void yawMOtorSet(ctrlOutYaw);
        void pitchMotorSet(double*);
        void rollMotorSet(double*);
        // void thrustMotorSet(ctrlOutThrust);

        void init(void);
        void compute(void);

        // TRANSLATE MOTOR THRUST COMBINATIONS TO THRUST, ROLL, PITCH, DRONE
        
};

#endif