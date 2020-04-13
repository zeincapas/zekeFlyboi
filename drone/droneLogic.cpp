#include <Arduino.h>
#include <stdint.h>
#include <Arduino.h>
#include <stdbool.h>
#include <Wire.h>
#include <math.h>
#include "PID_v1.h"
#include "droneLogic.h"
#include "motor.h"
#include "MadgwickAHRS.h"
#include "MPU9250.h"

//Struct pointer instantiation and initializations. Store operation values here.
struct SensorData sensor;
SensorData* dataPtr = &sensor;

int statusIMU;
float roll, pitch;

const int MAX_PITCH = 180;
const int MAX_ROLL = 180;
const int MAX_YAW = 180;
const int HOVER_THRUST = 100;
const int FORCE_MAG_LOW_THRESH = 4.9;
const int FORCE_MAG_HIGH_THRESH = 18.6;

MPU9250 IMU(Wire, 0x68);
Madgwick Filter;
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
    IMU.begin();
    if (statusIMU < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(statusIMU);
        while(1) {}
    }
    Filter.begin(10); //10Khz since the default sample rate of the MPU is 1Khz
}

//This function updates the RAW values of the data read from the IMU.
void Drone::readSensorVal() 
{
    IMU.readSensor();
    
    dataPtr->rawGyro[0] = IMU.getGyroX_rads(); //ROLL: must be integrated to get proper angle
    dataPtr->rawGyro[1] = IMU.getGyroY_rads(); //PITCH: must be integrated to get proper angle
    dataPtr->rawGyro[2] = IMU.getGyroZ_rads(); //YAW: must be integrated to get proper angle 
    
    dataPtr->rawAccel[0] = IMU.getAccelX_mss(); //Acceleration in X direction
    dataPtr->rawAccel[1] = IMU.getAccelY_mss(); //Acceleration in Y direction
    dataPtr->rawAccel[2] = -IMU.getAccelZ_mss(); //Acceleration vector in the Z direction (must add up to 9.8 when stationary)
}

void Drone::updateAHRS()
{
    Filter.updateIMU(dataPtr->rawGyro[0], dataPtr->rawGyro[1], dataPtr->rawGyro[2],
                     dataPtr->rawAccel[0], dataPtr->rawAccel[1], dataPtr->rawAccel[2]);
    
    pitch = Filter.getPitch();
    roll = Filter.getRoll();
    
    // Serial.print("Pitch: "); Serial.println(pitch);
    // Serial.print("Roll: "); Serial.println(roll);
}

void Drone::printData()
{
}

