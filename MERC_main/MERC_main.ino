#include <Motor.h>
#include <RelayArray.h>

#include <DC_Servo.h>
#include <Hi229.h>

#include <PID_Control.h>


#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int maze_1_robot = 1;
const int maze_2_robot = 2;
const int maze_3_robot = 3;
#define robot_type      maze_1_robot

Motor motor_left = Motor(13, 15);   // dir, speed
Motor motor_right = Motor(19, 22);
Motor motor_hand = Motor(17, 16);
DC_Servo hand_servo = DC_Servo();
DC_Servo body_servo = DC_Servo();



PIDController forward_pid   = PIDController(20, 2, 1, 127);
Chassis dual_wheel = Chassis(motor_left, motor_right, forward_pid);
Relay_Array relay_array = Relay_Array(5, 18, 23);

#define pump_left       0
#define coil_left       1
#define xilanh          2
#define pump_front      3
#define coil_front      4


int wheel_speed = 0;
int duration = 0;
int angle = 0;
int speed_combo = 100;

int current_dir = 0;
void setup() {
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
        next_update = millis() + 45;
        // Serial.println(current_dir);
    }else{
        if(millis() > next_update){
            dual_wheel.run();
            next_update += 40;
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
//combo 2 function: take ball automatic
void take_ball() {
    if(robot_type == maze_1_robot) {
        //bật hút
        
        //hạ tay 
        servo_hand.goto_by_mm(int );
        //nâng tay
        servo_hand.goto_by_mm(int );

    } else if (robot_type == maze_2_robot) {
        //bật hút

        //hạ tay 
        servo_hand.goto_by_mm(int );
        //nâng tay
        servo_hand.goto_by_mm(int );

    } else if (robot_type == maze_3_robot) {
        //bật hút

        //hạ tay 
        body_servo.goto_by_mm(int );
        delay(1000);
        //nâng tay
        body_servo.goto_by_mm(int );

    }
}
//combo 3 function: drop ball automatic
void drop_ball() {
    if(robot_type == maze_1_robot) {
        //act 1: hạ tay
        body_servo.goto_by_mm(int );
        //act 2: lui
        
        
    } else if (robot_type == maze_2_robot) {
        //act 1: hạ tay
        body_servo.goto_by_mm(int );
        //act 2: lui
        dual_wheel.set_speed(-100);
        dual_wheel.set_speed(0);
        dual_wheel.set_speed(0);
    } else if (robot_type == maze_3_robot) {
        //act 1: hạ tay
        body_servo.goto_by_mm(int );
        //act 2: lui
        
    }
}
//combo 4 function: hút quân lương bán tự động 
void suck_box() {
    if(robot_type == maze_1_robot) {
        //act 1: bật hút
        relay_array.set_status(coil_left, 0);
        relay_array.set_status(pump_left, 1);
        //act 2: hạ tay xuống hết + 
        body_servo.goto_by_mm(int min_body);
        //act 3: dài tay sau 
        hand_servo.goto_by_mm(int max_hand);
        //act 4: thu tay sau
        hand_servo.goto_by_mm(int );
        //act 5: nâng body
        body_servo.goto_by_mm(int );
        
    } else if (robot_type == maze_2_robot) {
        //act 1: bật hút
        relay_array.set_status(coil_left, 0);
        relay_array.set_status(pump_left, 1);
        //act 2: hạ tay xuống hết + 
        body_servo.goto_by_mm(int min_body);
        //act 3: dài tay sau 
        hand_servo.goto_by_mm(int max_hand);
        //act 4: thu tay sau
        hand_servo.goto_by_mm(int );
        //act 5: nâng body
        body_servo.goto_by_mm(int );
        
    } else if (robot_type == maze_3_robot) {
        //act 1: bật hút
        relay_array.set_status(coil_left, 0);
        relay_array.set_status(pump_left, 1);
        //act 2: hạ tay xuống hết + 
        body_servo.goto_by_mm(int min_body);
        //act 3: dài tay sau 
        hand_servo.goto_by_mm(int max_hand);
        //act 4: thu tay sau
        hand_servo.goto_by_mm(int );
        //act 5: nâng body
        body_servo.goto_by_mm(int );
        
    }
}

//combo 5 function: kẹp cọc + thả cọc 
void pile_clamp() {
    if(robot_type == maze_1_robot) {
        //act 1: kẹp xilanh
        relay_array.set_status(xilanh, 1);
        //act 2: keep forward
        dual_wheel.keep_forward(  );
        //act 3: thả xilanh
        relay_array.set_status(xilanh, 0);
        //act 4: tự động lui
        dual_wheel.keep_forward(- );
        
    } else if (robot_type == maze_2_robot) {
        //act 1: kẹp xilanh
        relay_array.set_status(xilanh, 1);
        //act 2: keep forward
        dual_wheel.keep_forward(  );
        //act 3: thả xilanh
        relay_array.set_status(xilanh, 0);
        //act 4: tự động lui
        dual_wheel.keep_forward(- );
        
    } else if (robot_type == maze_3_robot) {
        //act 1: kẹp xilanh
        relay_array.set_status(xilanh, 1);
        //act 2: keep forward
        dual_wheel.keep_forward(  );
        //act 3: thả xilanh
        relay_array.set_status(xilanh, 0);
        //act 4: tự động lui
        dual_wheel.keep_forward(- );
        
    }
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
        if(value == 1)  combo_1();
        if(value == 2)  take_ball();
        if(value == 3)  drop_ball();
        if(value == 4)  suck_box();
        if(value == 5)  pile_clamp();


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

//COMBO Function
