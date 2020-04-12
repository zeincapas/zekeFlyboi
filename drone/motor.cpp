#include <Arduino.h>
#include <stdbool.h>
#include <Servo.h>
#include "motor.h"

Servo motor;

void BLDC::init() 
{
    motor.attach(motorPin, 1000, 2000);
}

void BLDC::setSpeed(int32_t speed) 
{
    motor.write(speed);
}