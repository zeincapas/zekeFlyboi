#include "RF24.h"
#include "nRF24L01.h"

RF24 radio(7,8); // CE, CSN
const byte address = "00001";

void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void loop() {
    const char text[] = "Hello World";
    if(!radio.write(&text, sizeof(text))) {
        Serial.println("Failed");
    }
    else {
        Serial.println("Success");
    }
    delay(1000);
}






