#include "config.h"
#include "wheel.h"


#include "BluetoothSerial.h"
BluetoothSerial SerialBT;




Motor motor_left = Motor(1, 2);
Motor motor_right = Motor(3, 4);
Chassis dual_wheel = Chassis(motor_left, motor_right);



void setup(){
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 15, 16);  // Serial2 for UART input
    
    Serial.println("ESP32 UART listening...");
    SerialBT.begin("Maze"); // Set the Bluetooth device name
}


void keep_forward(int distance=120){

}


void loop(){
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processSerialCommand(command);
    }

    // Check for serial commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        processSerialCommand(command);
    }
}



void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();
    
}
