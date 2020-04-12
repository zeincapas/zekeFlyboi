#include <Arduino.h>
#include <stdint.h>
#include <Arduino.h>
#include <stdbool.h>
#include <Wire.h>
#include "PID_v1.h"
#include "droneLogic.h"
#include "motor.h"
#include "src/FreeSixIMU/FreeSixIMU.h"
#include "src/FreeSixIMU/FIMU_ADXL345.h"
#include "src/FreeSixIMU/FIMU_ITG3200.h"

const static double pitchGain[3] = {0, 0, 0}; //Kp, Ki, Kd
const static double rollGain[3] = {0, 0, 0};  //Kp, Ki, Kd

const static int MAX_PITCH = 180;
const static int MAX_ROLL = 180;

const static int HOVER_THRUST = 100;

float angles[3];  // yaw pitch roll

double currYaw, outputYaw, setYaw, currPitch, outputPitch, setPitch, currRoll, outputRoll, setRoll;

// PID thrust(&currThrust, &ouputThrust, &setThrust, Kp_Thrust, Ki_Thrust, Kd_Thrust, REVERSE);
// PID yaw(&currYaw, &outputYaw, &setYaw, Kp_Yaw, Ki_Yaw, Kd_Yaw, REVERSE);
PID pitchPID(&currPitch, &outputPitch, &setPitch, pitchGain[0], pitchGain[1], pitchGain[2], REVERSE);
PID rollPID(&currRoll, &outputRoll, &setRoll, rollGain[0], rollGain[1], rollGain[2], REVERSE);
FreeSixIMU sixDOF = FreeSixIMU();
BLDC motorFL(2);
BLDC motorFR(3);
BLDC motorBL(4);
BLDC motorBR(5);

/*******************************************************************************/
setPitch = 0;       //At this stage, we are only testing hover mode.
setRoll = 0;
/*******************************************************************************/

void Drone::init() 
{
    Serial.begin(9600);
    Wire.begin();
    // thrust.SetOutputLimits(-MAX_THRUST, MAX_THRUST);
    // yaw.SetOutputLimits(-MAX_YAW, MAX_YAW);
    pitchPID.SetOutputLimits(-MAX_PITCH, MAX_PITCH);
    rollPID.SetOutputLimits(-MAX_ROLL, MAX_ROLL);
    delay(10);
    sixDOF.init();
    delay(10);
}

void Drone::compute() 
{
    sixDOF.getEuler(angles);
    currPitch = angles[1];
    currRoll = angles[2];
    pitchPID.Compute();
    rollPID.Compute();
}

void Drone::pitchMotorSet(double* outputPitch) 
{
    ;
}



