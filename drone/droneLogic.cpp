#include <Arduino.h>
#include <stdint.h>
#include <Arduino.h>
#include <stdbool.h>
#include <Wire.h>
#include <math.h>
#include "PID_v1.h"
#include "droneLogic.h"
#include "motor.h"
#include "MPU9250.h"

//Struct pointer instantiation and initializations. Store operation values here.
struct Yaw yawData;
struct Pitch pitchData;
struct Roll rollData;
Yaw* yPtr = &yawData;
Pitch* pPtr = &pitchData;
Roll* rPtr = &rollData;

//Raw gyroscope values must be stored here. We are using pointers.
float rawYawGyro, rawPitchGyro, rawRollGyro;
float* pYawGyro = &rawYawGyro;
float* pPitchGyro = &rawPitchGyro;
float* pRollGyro = &rawRollGyro;

//Raw acceleration values must be stored here. We are using pointers.
float rawZAcc, rawPitchAcc, rawRollAcc;
float* pZAcc = &rawZAcc;
float* pPitchAcc = &rawPitchAcc;
float* pRollAcc = &rawRollAcc;

float pitchAccuracyDeg, rollAccuracyDeg, pitchDeg, rollDeg;
int statusIMU;

float henlo = 0;

const float pi = 3.14159265359;
const float dt = 0.0014; 
const int MAX_PITCH = 180;
const int MAX_ROLL = 180;
const int MAX_YAW = 180;
const int HOVER_THRUST = 100;
const int FORCE_MAG_LOW_THRESH = 4.9;
const int FORCE_MAG_HIGH_THRESH = 9.8;

// PID thrust(&currThrust, &ouputThrust, &setThrust, Kp_Thrust, Ki_Thrust, Kd_Thrust, REVERSE);
PID yawPID(&(yPtr->curr), &(yPtr->out), &(yPtr->set), yPtr->Kp, yPtr->Ki, yPtr->Kd, REVERSE);
PID pitchPID(&(pPtr->curr), &(pPtr->out), &(pPtr->set), pPtr->Kp, pPtr->Ki, pPtr->Kd, REVERSE);
PID rollPID(&(rPtr->curr), &(rPtr->out), &(rPtr->set), rPtr->Kp, rPtr->Ki, rPtr->Kd, REVERSE);
MPU9250 IMU(Wire, 0x68);
BLDC motorFL(2);
BLDC motorFR(3);
BLDC motorBL(4);
BLDC motorBR(5);

// /*******************************************************************************/
// setPitch = 0;       //At this stage, we are only testing hover mode.
// setRoll = 0;
// /*******************************************************************************/

void Drone::init() 
{
    Serial.begin(115200);
    while(!Serial) {}
    // thrust.SetOutputLimits(-MAX_THRUST, MAX_THRUST);
    yawPID.SetOutputLimits(-MAX_YAW, MAX_YAW);
    pitchPID.SetOutputLimits(-MAX_PITCH, MAX_PITCH);
    rollPID.SetOutputLimits(-MAX_ROLL, MAX_ROLL);

    IMU.begin();
    if (statusIMU < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(statusIMU);
        while(1) {}
    }

}

//This function updates the RAW values of the data read from the IMU.
void Drone::readSensorVal() 
{
    IMU.readSensor();
    *pYawGyro = IMU.getGyroZ_rads(); //YAW: must be integrated to get proper angle 
    *pPitchGyro = IMU.getGyroY_rads(); //PITCH: must be integrated to get proper angle
    *pRollGyro = IMU.getGyroX_rads(); //ROLL: must be integrated to get proper angle
    *pZAcc = IMU.getAccelZ_mss(); //Acceleration vector in the Z direction (must add up to 9.8 when stationary)
    *pPitchAcc = IMU.getAccelX_mss(); //Acceleration in X direction
    *pRollAcc = IMU.getAccelY_mss(); //Acceleration in Y direction
}

void Drone::complementaryFilter() 
{
    //Integrate the gyroscope data and then translate them to degrees.
    
    henlo += ((*pPitchGyro * dt)) * (180/pi);
    // rollDeg = (rPtr->curr + (*pRollGyro * dt)) * (180/pi);
    Serial.println(henlo);
    // float forceMagnitude = abs(*pZAcc) + abs(*pPitchAcc) + abs(*pRollAcc);
    // if ((forceMagnitude > FORCE_MAG_LOW_THRESH) && (forceMagnitude < FORCE_MAG_HIGH_THRESH))
    // {   
    //     //Find pitch angle in degrees.
    //     pitchAccuracyDeg = atan2(*pPitchAcc, *pZAcc) * (180/pi);
    //     pPtr->curr = (0.98 * pitchDeg) + (0.02 * (pitchAccuracyDeg));

    //     //Find roll angle in degrees.
    //     rollAccuracyDeg = atan2(*pRollAcc, *pZAcc) * (180/pi);
    //     rPtr->curr = (0.98 * rollDeg) + (0.02 * (rollAccuracyDeg));
    // }
    
}

void Drone::printData()
{
    Serial.print("Pitch Angle: "); Serial.println(pPtr->curr, 6);
    Serial.print("Roll Angle: "); Serial.println(rPtr->curr, 6);
}



