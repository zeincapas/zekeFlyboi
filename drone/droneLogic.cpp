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

double currentPitch, ctrlOutPitch, setPitch, currentRoll, ctrlOutRoll, setRoll;



/*******************************************************************************/
setPitch = 0;       //At this stage, we are only testing hover mode.
setRoll = 0;
/*******************************************************************************/


// PID thrust(&currentThrust, &ouputThrust, &setThrust, Kp_Thrust, Ki_Thrust, Kd_Thrust, REVERSE);
// PID yaw(&currentYaw, &outputYaw, &setYaw, Kp_Yaw, Ki_Yaw, Kd_Yaw, REVERSE);
PID pitch(&currentPitch, &outputPitch, &setPitch, pitchGain[0], pitchGain[1], pitchGain[2], REVERSE);
PID roll(&currentRoll, &outputRoll, &setRoll, rollGain[0], rollGain[1], rollGain[2], REVERSE);
FreeSixIMU sixDOF = FreeSixIMU();
BLDC motorFL(2);
BLDC motorFR(3);
BLDC motorBL(4);
BLDC motorBR(5);

void Drone::init() {
    Serial.begin(9600);
    Wire.begin();
    // thrust.SetOutputLimits(-MAX_THRUST, MAX_THRUST);
    // yaw.SetOutputLimits(-MAX_YAW, MAX_YAW);
    pitch.SetOutputLimits(-MAX_PITCH, MAX_PITCH);
    roll.SetOutputLimits(-MAX_ROLL, MAX_ROLL);
    delay(10);
    sixDOF.init();
    delay(10);
}

void Drone::compute() {
    sixDOF.getEuler(angles);
    currentPitch = angles[1];
    currentRoll = angles[2];
    pitch.compute();
    roll.compute();
}

void Drone::pitchMotorSet() {
    
}



