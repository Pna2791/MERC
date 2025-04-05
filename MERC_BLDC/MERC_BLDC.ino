#include "motor.h"


#define speed_pin 13
#define dir_pin 15
Motor my_motor = Motor(dir_pin, speed_pin);

int motor_speed = 255;


void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 12, 14);  // Serial2 for UART input
}


void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
    // Check for serial commands
    if (Serial2.available()) {
        String command = Serial2.readStringUntil('\n');
        processSerialCommand(command);
    }
}


void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("B")) {
        int value = command.substring(1).toInt();
        my_motor.setSpeed(value);
    }
}