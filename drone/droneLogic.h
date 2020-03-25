#ifndef DRONELOGIC_H_
#define DRONELOGIC_H_

class Drone {
    public:
        void yawMOtorSet(ctrlOutYaw);
        void pitchMotorSet(ctrlOutPitch);
        // void rollMotorSet(ctrlOutRoll);
        // void thrustMotorSet(ctrlOutThrust);

        void init(void);
        void compute(void);

        // TRANSLATE MOTOR THRUST COMBINATIONS TO THRUST, ROLL, PITCH, DRONE
        
};

#endif