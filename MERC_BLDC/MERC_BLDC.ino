
#include <EncoderTimer.h>
#include <Encoder.h>
#include <Motor.h>
#include <DC_Servo.h>
#include <PID_Control.h>


Encoder body_encoder(33, 32);
Motor   body_motor(15, 13);     // dir, speed
PIDController   body_pid(0.9, 0, 0.003, 240);   // P, I, D, max_speed
DC_servo body_servo(body_motor, body_encoder, body_pid, 50);    // steps/mm


bool servo_stt = true;
void setup() {
    Serial.begin(115200);
    Serial1.begin(57600, SERIAL_8N1, 25, 26);  // Serial1 for UART input
    body_encoder.begin();
}


void loop() {
    // Show encoder count every second
    static long next_update = millis();
    if(millis() > next_update){
        if(servo_stt)   body_servo.run();
        next_update += INTERVAL;
    }

    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }
    // Check for serial commands
    if (Serial1.available()) {
        String command = Serial1.readStringUntil('\n');
        processSerialCommand(command);
    }
}


void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("B")) {
        servo_stt = false;
        int value = command.substring(1).toInt();
        body_motor.setSpeed(value);
    }
    if (command.startsWith("T")) {
        servo_stt = true;
        int value = command.substring(1).toInt();
        body_servo.goto_position_mm(value);
    }
    if (command.startsWith("R")) {
        int value = command.substring(1).toInt();
        body_servo.reset(value);
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