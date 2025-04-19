#define DEBUG

#include <Encoder.h>
#include <Motor.h>
#include <DC_Servo.h>


// Encoder body_encoder(35, 34);
// Motor   body_motor(23, 18);
// PIDController   body_pid(0.9, 0, 0.003, 240);
// DC_servo body_servo(body_motor, body_encoder, body_pid, 50);

Encoder body_encoder(35, 34);
Motor   body_motor(17, 16);
PIDController   body_pid(10, 0, 0.3, 255, 5);  // P, I, D, max speed, skip error
DC_servo body_servo(body_motor, body_encoder, body_pid, 5);


void setup(){
    Serial.begin(115200);
    body_encoder.begin();
}


void loop(){
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
    static long next_update = millis();
    if(millis() > next_update){
        body_servo.run();
        next_update += INTERVAL;
    }
}



void processSerialCommand(String command) {
    command.trim();
    
    if (command.startsWith("B")) {
        int value = command.substring(1).toInt();
        body_motor.setSpeed(value);
    }
    if (command.startsWith("T")) {
        int value = command.substring(1).toInt();
        body_servo.goto_position_mm(value);
    }
    if (command.startsWith("P")) {
        float value = command.substring(1).toFloat();
        body_pid.set_P(value);
    }
    if (command.startsWith("I")) {
        float value = command.substring(1).toFloat();
        body_pid.set_I(value);
    }
    if (command.startsWith("D")) {
        float value = command.substring(1).toFloat();
        body_pid.set_D(value);
    }
}
