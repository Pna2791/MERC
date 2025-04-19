#define SHOW_ENCODER

#define ROBOT_NAME_1    "Maze1"
// #define ROBOT_NAME_2    "Maze2"
// #define ROBOT_NAME_3    "Maze3"


#include <Encoder.h>
#include <Motor.h>
#include <DC_Servo.h>
#include <RelayArray.h>
#include <Hi229.h>

#include <PID_Control.h>




#include "BluetoothSerial.h"
BluetoothSerial SerialBT;



#ifdef ROBOT_NAME_1             
    #define wheel_step_per_mm 1
#elif defined(ROBOT_NAME_2)     
    #define wheel_step_per_mm 2.65
#elif defined(ROBOT_NAME_2)     
    #define wheel_step_per_mm 2.65
#endif
Motor motor_left = Motor(13, 15);   // dir, speed
Motor motor_right = Motor(19, 22);
Encoder wheel_encoder(34, 35);


PIDController forward_pid   = PIDController(20, 2, 1, 127);
Chassis dual_wheel = Chassis(motor_left, motor_right, forward_pid);
Relay_Array relay_array = Relay_Array(5, 18, 23);

Encoder hand_encoder(32, 33);
Motor   hand_motor(17, 16);
PIDController   hand_pid(10, 0, 0.3, 255, 5);  // P, I, D, max speed, skip error
DC_servo hand_servo(hand_motor, hand_encoder, hand_pid, 5);



#define pump_left       0
#define coil_left       1

int wheel_speed = 0;
int duration = 0;
int angle = 0;


int current_dir = 0;
void setup() {
    hand_encoder.begin();
    wheel_encoder.begin();

    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 12, 14);
    // Serial2.begin(115200, SERIAL_8N1, 14, 12);
    Serial1.begin(9600, SERIAL_8N1, 26, 25);
    SerialBT.begin("Maze2"); // Set the Bluetooth device name
}
void reset_IMU(){
    Serial2.println("AT+RST");
    delay(1000);
    Serial2.println("AT+RST");
}

void servo_run(){
    Serial.println(wheel_encoder.getCount());
}


#define AUTO_FW_SPEED   127
void auto_forward(int length=1200, int time_out=3000){
    delay(1000);
    long target_pos = wheel_encoder.getCount() + wheel_step_per_mm*length;
    time_out += millis();

    dual_wheel.set_speed(AUTO_FW_SPEED, AUTO_FW_SPEED, 15);
    while(millis() < time_out && wheel_encoder.getCount() < target_pos){
        dual_wheel.run();
        delay(INTERVAL);
    }
    dual_wheel.stop();
}

void loop() {
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

    // Check for serial commands
    if (SerialBT.available()) {
        String command = SerialBT.readStringUntil('\n');
        processSerialCommand(command);
    }

    static long next_update = millis();
    int received_dir = get_direction(Serial2);
    if(received_dir != 0xFFF){
        current_dir = received_dir;
        if(dual_wheel.code == 1)
            dual_wheel.keep_forward(received_dir, wheel_speed, SerialBT);

        dual_wheel.run();
        next_update = millis() + INTERVAL;
        servo_run();
        // Serial.println(current_dir);
    }else{
        if(millis() > next_update+5){
            servo_run();
            dual_wheel.run();
            next_update += INTERVAL;
        }
    }

}

void move_wheel(int dir){
    dual_wheel.code = dir;
    if(dir == 0){
        dual_wheel.stop();
        return;
    }
    if(dir == 1){
        forward_pid.reset();
        dual_wheel.set_target_dir(current_dir);
        dual_wheel.set_speed(wheel_speed, wheel_speed, 15);
        return;
    }
    if(dir == 2){
        dual_wheel.set_speed(-wheel_speed, -wheel_speed, 5);
        return;
    }
    if(dir == 3){
        dual_wheel.set_speed(-wheel_speed/2, wheel_speed/2, 1);
        return;
    }
    if(dir == 4){
        dual_wheel.set_speed(wheel_speed/2, -wheel_speed/2, 1);
        return;
    }
    if(dir == 5){
        dual_wheel.set_speed(0, wheel_speed, 1);
        return;
    }
    if(dir == 6){
        dual_wheel.set_speed(wheel_speed, 0, 1);
        return;
    }
    if(dir == 7){
        dual_wheel.set_speed(0, -wheel_speed, 1);
        return;
    }
    if(dir == 8){
        dual_wheel.set_speed(-wheel_speed, 0, 1);
        return;
    }
}

void test_rotate(){
    motor_left.setSpeed(-255);
    motor_right.setSpeed(255);
    delay(duration*100);
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
}

void take(){
    relay_array.set_status(coil_left, 0);
    relay_array.set_status(pump_left, 1);
}

void drop(){
    relay_array.set_status(coil_left, 1);
    relay_array.set_status(pump_left, 0);
}

void combo_1(){
    delay(2000);
    // long time_out = millis() + 100*duration;
    if(angle > 0){
        motor_left.setSpeed(wheel_speed);
        motor_right.setSpeed(wheel_speed*(1-float(angle)/100.0));
    }else{
        motor_left.setSpeed(wheel_speed*(1+float(angle)/100));
        motor_right.setSpeed(wheel_speed);
    }
    delay(long(100)*duration);
    motor_left.setSpeed(0);
    motor_right.setSpeed(0);
    SerialBT.discoverClear();
}

void update_k_PID(String command){
    float value = command.substring(1).toFloat();

    if (command.startsWith("P")) {
        forward_pid.set_P(value);
    }
    if (command.startsWith("I")) {
        forward_pid.set_I(value);
    }
    if (command.startsWith("D")) {
        forward_pid.set_D(value);
    }
}

void process_combo(int value){
    if(value == 1)  combo_1();
    if(value == 23)  auto_forward(1200);
}

#define body_speed 255
#define hand_speed 255
void processSerialCommand(String command) {
    Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if (command.startsWith("B")) {
        char ch = command.charAt(1);
        if(ch == '0')   Serial1.println("B0");
        if(ch == '+')   Serial1.println("B" + String(int(body_speed/4)));
        if(ch == '-')   Serial1.println("B-" + String(int(body_speed/4)));
        if(ch == '*')   Serial1.println("B" + String(body_speed));
        if(ch == '/')   Serial1.println("B-" + String(body_speed));
    }

    if (command.startsWith("H")) {
        char ch = command.charAt(1);
        if(ch == '0')   hand_motor.setSpeed(0);
        if(ch == '+')   hand_motor.setSpeed(hand_speed/4);
        if(ch == '-')   hand_motor.setSpeed(-hand_speed/4);
        if(ch == '*')   hand_motor.setSpeed(hand_speed);
        if(ch == '/')   hand_motor.setSpeed(-hand_speed);
    }

    if (command.startsWith("S")) {
        int value = command.substring(2).toInt();
        wheel_speed = value;
    }
    if (command.startsWith("M")) {
        int value = command.substring(1).toInt();
        move_wheel(value);
        dual_wheel.run();
    }

    if (command.startsWith("k")) {
        update_k_PID(command.substring(1));
    }
    
    if (command.startsWith("T")) {
        int value = command.substring(1).toInt();
        if(value == 0)  drop();
        else            take();
    }
    
    if (command.startsWith("C")) {
        int value = command.substring(1).toInt();
        process_combo(value);
    }

    if (command.startsWith("A")) {
        angle = command.substring(1).toInt()-20;
    }
    if (command.startsWith("D")) {
        duration = command.substring(1).toInt();
    }
    if (command.startsWith("R")) {
        test_rotate();
    }
    

}