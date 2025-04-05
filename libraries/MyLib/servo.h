
#ifndef SERVO_H
#define SERVO_H

#include "encoder.h"
#include "wheel.h"


class DC_Servo{
private:
    Motor &motor;
    Encoder &encoder;
public:
    DC_Servo(Motor &motor, Encoder &encoder) {
        this->motor = motor;
        this->encoder = encoder; 
    }
}

#endif
