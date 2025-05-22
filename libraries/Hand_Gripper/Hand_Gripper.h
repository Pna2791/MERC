#ifndef HAND_GRIPPER_H
#define HAND_GRIPPER_H


#include <ESP32Servo.h>
#include <EEPROM.h>

#define EEPROM_SIZE 32

class Hand_Gripper {
    private:
        Servo gripperServo;
        int pin;

        int offset = 0;
        int openPos = 0;
        int pipePos = 85;
        int ballPos = 65;
        int foodPos = 50;

    public:
        Hand_Gripper(int servoPin) {
            pin = servoPin;
        }

        void begin() {
            EEPROM.begin(EEPROM_SIZE);  // Initialize EEPROM
            // ESP32PWM::allocateTimer(3);

            gripperServo.setPeriodHertz(50);
            gripperServo.attach(pin, 500, 2500);
            // pinMode(pin, OUTPUT);

            offset = EEPROM.read(10);
            foodPos = EEPROM.read(11);
            ballPos = EEPROM.read(12);
            pipePos = EEPROM.read(13);

            open();
            debug_print();
        }

        void servo_write(int value = 0) {
            int corrected = constrain(90 + offset - value, 0, 180);
            gripperServo.write(corrected);
            // custom_write(corrected);
        }

        void custom_write(int angle){
            int pulseWidth = map(angle, 0, 180, 500, 2500);  // Map angle to pulse width
            digitalWrite(pin, HIGH);
            delayMicroseconds(pulseWidth);  // Send the pulse
            digitalWrite(pin, LOW);
        }

        void set_offset(int value=0){
            offset = value;
            EEPROM.write(10, value);  // Save offset to EEPROM address 10
            EEPROM.commit();  // Ensure data is written
            servo_write(0);
        }
        void set_food(int value=0){
            foodPos = value;
            EEPROM.write(11, value);  // Save offset to EEPROM address 10
            EEPROM.commit();  // Ensure data is written
            servo_write(foodPos);
        }
        void set_ball(int value=0){
            ballPos = value;
            EEPROM.write(12, value);  // Save offset to EEPROM address 10
            EEPROM.commit();  // Ensure data is written
            servo_write(ballPos);
        }
        void set_pipe(int value=0){
            pipePos = value;
            EEPROM.write(13, value);  // Save offset to EEPROM address 10
            EEPROM.commit();  // Ensure data is written
            servo_write(pipePos);
        }


        void open() {
            servo_write(0);
        }

        void take_pipe() {
            servo_write(pipePos);
        }

        void take_ball() {
            servo_write(ballPos);
        }

        void take_food() {
            servo_write(foodPos);
        }

        void debug_print() {
            Serial.println("--- Gripper Settings ---");
            Serial.printf("Offset: %d\n", offset);
            Serial.printf("Open: %d\n", openPos);
            Serial.printf("Pipe: %d\n", pipePos);
            Serial.printf("Ball: %d\n", ballPos);
            Serial.printf("Food: %d\n", foodPos);
        }
};




#endif