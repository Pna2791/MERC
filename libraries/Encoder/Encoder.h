#ifndef ENCODER_H
#define ENCODER_H

#include "attachInterruptEx.h"


class Encoder {
public:
    Encoder(int interruptPin, int directionPin)
        : pinA(interruptPin), pinB(directionPin) {
        }
    void begin(){
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterruptEx(pinA, [this] { this -> ISR(); }, FALLING);
        Serial.println("Started");
    }
    long getCount() {
        return encoderCount;
    }

private:
    int pinA;
    int pinB;
    volatile long encoderCount = 0;

    void ISR() {
        encoderCount += !digitalRead(pinB) ? 1 : -1;
    }
};


#endif
