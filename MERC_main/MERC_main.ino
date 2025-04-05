#include "motor.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;



Motor motor_left = Motor(13, 15);   // dir, speed
Motor motor_right = Motor(19, 22);
Motor motor_hand = Motor(17, 16);


int latchPin = 5;
int clockPin = 18;
int dataPin = 23;


int wheel_speed = 0;


void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 14, 12);  // Serial2 for UART input
    SerialBT.begin("Maze2"); // Set the Bluetooth device name

    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
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

    // Check for serial commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        processSerialCommand(command);
    }
}

void move_wheel(int dir){
    if(dir == 0){
        motor_left.setSpeed(0);
        motor_right.setSpeed(0);
        return;
    }
    if(dir == 1){
        motor_left.setSpeed(wheel_speed);
        motor_right.setSpeed(wheel_speed);
        return;
    }
    if(dir == 2){
        motor_left.setSpeed(-wheel_speed);
        motor_right.setSpeed(-wheel_speed);
        return;
    }
    if(dir == 3){
        motor_left.setSpeed(-wheel_speed/2);
        motor_right.setSpeed(wheel_speed/2);
        return;
    }
    if(dir == 4){
        motor_left.setSpeed(wheel_speed/2);
        motor_right.setSpeed(-wheel_speed/2);
        return;
    }
    if(dir == 5){
        motor_left.setSpeed(0);
        motor_right.setSpeed(wheel_speed/2);
        return;
    }
    if(dir == 6){
        motor_left.setSpeed(wheel_speed/2);
        motor_right.setSpeed(0);
        return;
    }
    if(dir == 7){
        motor_left.setSpeed(0);
        motor_right.setSpeed(-wheel_speed/2);
        return;
    }
    if(dir == 8){
        motor_left.setSpeed(-wheel_speed/2);
        motor_right.setSpeed(0);
        return;
    }
}



void take(){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 0);  // Turn off all LEDs
    digitalWrite(latchPin, HIGH);
    delay(100);

    byte numberToDisplay = 1 << 0;  // Shift 1 to the correct bit position
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, numberToDisplay);
    digitalWrite(latchPin, HIGH);
}

void drop(){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 0);  // Turn off all LEDs
    digitalWrite(latchPin, HIGH);
    delay(100);

    byte numberToDisplay = 1 << 1;  // Shift 1 to the correct bit position
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, numberToDisplay);
    digitalWrite(latchPin, HIGH);
}


#define body_speed 255
#define hand_speed 255
void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("B")) {
        char ch = command.charAt(1);
        if(ch == '0')   Serial2.println("B0");
        if(ch == '+')   Serial2.println("B" + String(int(body_speed/4)));
        if(ch == '-')   Serial2.println("B-" + String(int(body_speed/4)));
        if(ch == '*')   Serial2.println("B" + String(body_speed));
        if(ch == '/')   Serial2.println("B-" + String(body_speed));
    }

    if (command.startsWith("H")) {
        char ch = command.charAt(1);
        if(ch == '0')   motor_hand.setSpeed(0);
        if(ch == '+')   motor_hand.setSpeed(hand_speed/4);
        if(ch == '-')   motor_hand.setSpeed(-hand_speed/4);
        if(ch == '*')   motor_hand.setSpeed(hand_speed);
        if(ch == '/')   motor_hand.setSpeed(-hand_speed);
    }

    if (command.startsWith("S")) {
        int value = command.substring(2).toInt();
        wheel_speed = value;
    }
    if (command.startsWith("M")) {
        int value = command.substring(1).toInt();
        move_wheel(value);
    }
    
    if (command.startsWith("T")) {
        int value = command.substring(1).toInt();
        if(value == 0)  drop();
        else            take();
    }

}