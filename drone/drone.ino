#include <Arduino.h>
#include <Servo.h>
#include "droneLogic.h"

Drone drone;
// #define POT_SIG_PIN A0
// #define MOTOR_SIG_PIN 3

// Servo brushless;

// uint32_t read = 0;

// void setup() 
// {
//     Serial.begin(9600);
//     pinMode(POT_SIG_PIN, INPUT);
//     brushless.attach(MOTOR_SIG_PIN, 1000, 2000);
// }

// void loop() 
// {
//     read = analogRead(POT_SIG_PIN);
//     read = map(read, 0, 1023, 0, 180);
//     brushless.write(read);
// }

void setup()
{
    drone.init();
}

void loop()
{
    drone.readSensorVal();
    drone.updateAHRS();
    // drone.printData();
}