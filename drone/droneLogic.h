#ifndef DRONELOGIC_H_
#define DRONELOGIC_H_

struct SensorData
{
    float rawAccel[3];
    float rawGyro[3];
    float rawMag[3];       
};

class Drone 
{
    private:

    public:
        // void yawMOtorSet(ctrlOutYaw);
        // void pitchMotorSet(double*);
        // void rollMotorSet(double*);
        void readSensorVal(void);
        void updateAHRS(void);
        // void thrustMotorSet(ctrlOutThrust);

        void init(void);
        void printData(void);
        // void compute(void);

        // TRANSLATE MOTOR THRUST COMBINATIONS TO THRUST, ROLL, PITCH, DRONE
        
};

#endif