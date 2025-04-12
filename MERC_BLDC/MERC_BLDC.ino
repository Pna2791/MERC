#include "motor.h"


// #define speed_pin 13
// #define dir_pin 15
// #define encoder_A   33
// #define encoder_B   32

// #define speed_pin   18
// #define dir_pin     23
// #define encoder_A   35
// #define encoder_B   34

#define speed_pin 22
#define dir_pin 19
#define encoder_A   39
#define encoder_B   36


Motor my_motor = Motor(dir_pin, speed_pin);

int motor_speed = 255;

volatile long encoder_count = 0;
void IRAM_ATTR encoderA_ISR() {
    // Read encoder_B to determine direction
    if (digitalRead(encoder_B)) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 12, 14);  // Serial2 for UART input
    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder_A), encoderA_ISR, RISING); // or CHANGE/FALLING depending on your encoder
}


void loop() {
    // Show encoder count every second
    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        last_print = millis();
        Serial.print("Encoder Count: ");
        Serial.println(encoder_count);
        encoder_count = 0;
    }

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