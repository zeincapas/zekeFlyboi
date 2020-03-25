#include <Arduino.h>
#include <stdbool.h>
#include "wiredComms.h"

void Wired::init() {
    pinMode(wirePin, INPUT);
}

int32_t Wired::receive() {
    readValue = analogRead(wirePin);
    readValue = map(readValue, 0, 1023, 0, 180);
    return readValue;
}