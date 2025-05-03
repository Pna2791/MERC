#define SHOW_ENCODER


#include <EncoderTimer.h>
#include <Motor.h>
#include <DC_Servo.h>
#include <RelayArray.h>
#include <Hi229.h>

#include <PID_Control.h>

// #define DEBUG


#include "BluetoothSerial.h"
BluetoothSerial SerialBT;



#define ROBOT_NAME_1    1
#define ROBOT_NAME_2    2
#define ROBOT_NAME_3    3

#define robot_type ROBOT_NAME_1

#if robot_type == ROBOT_NAME_1           
    #define wheel_step_per_mm 1
    #define ROBOT_NAME          "Maze1"
#elif robot_type == ROBOT_NAME_2  
    #define wheel_step_per_mm 2.65
    #define ROBOT_NAME          "Maze2"
#elif robot_type == ROBOT_NAME_3   
    #define wheel_step_per_mm 2.65
    #define ROBOT_NAME          "Maze3"
#else
  #error "Unknown robot type"
#endif


Motor motor_left = Motor(13, 15, 1);   // dir, speed, direction(CV or CCV)
Motor motor_right = Motor(19, 22, -1);
// Encoder wheel_encoder(34, 35);
EncoderTimer wheel_encoder(34, 35, 0, 300);

PIDController forward_pid   = PIDController(10, 2, 1, 192); // 315rpm speed 100-150
PIDController rotate_pid    = PIDController(0.5, 0.0, 0.035, 192); // 315rpm speed 100-150
Chassis dual_wheel          = Chassis(motor_left, motor_right, forward_pid);
Relay_Array relay_array     = Relay_Array(5, 18, 23);

EncoderTimer hand_encoder(32, 33, 0, 300);
Motor   hand_motor(17, 16);
PIDController   hand_pid(10, 0, 0.3, 255, 5);  // P, I, D, max speed, skip error
DC_servo hand_servo(hand_motor, hand_encoder, hand_pid, 5);


//index: 0 1 2 6 7 5 4 3
#define pump_left       0
// #define coil_left       1
#define fan_left        1
#define xilanh          2
#define fan_right       6
#define pump_right      7
// #define coil_right      4


//define 
#define left_mode       0
#define right_mode      1    
int yard = right_mode;

int wheel_speed = 0;
int duration = 0;
int angle = 0;
bool servo_stt = false;



int current_dir = 0;
void setup() {
    hand_encoder.begin();
    wheel_encoder.begin();
    relay_array.set_status(xilanh, 0);
    

    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 12, 14);
    // Serial2.begin(115200, SERIAL_8N1, 14, 12);
    Serial1.begin(57600, SERIAL_8N1, 26, 25);
    SerialBT.begin(ROBOT_NAME); // Set the Bluetooth device name
    Serial.println("Started");
    relay_array.reset();
}
void reset_IMU(){
    Serial2.println("AT+RST");
    delay(1000);
    Serial2.println("AT+RST");
}

void servo_run(){
    if(!servo_stt){
        return;
    }
    hand_servo.run();
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

void rotate_CCW(int target_dir){
    target_dir = (target_dir + 3600)%3600;

    rotate_pid.reset();
    int mean_error = 900;

    long time_out = millis();
    if(abs(current_dir-target_dir)<1200)    time_out += 600;
    else                                    time_out += 900;
    while(millis() < time_out){
        int direction = get_direction(Serial2);
        if(direction != 0xFFF){
            mean_error = 0.7*mean_error + 0.3*(abs(target_dir-direction)%3600);
            if(mean_error < 10)        break;

            if(abs(direction - target_dir) > 1800)  direction += 3600;

            int turn_value = rotate_pid.compute(target_dir, direction);
            dual_wheel.rotate_CCW(turn_value);
            
            #ifdef DEBUG
                String message = String(turn_value/255.) + '\t' + String(target_dir-direction);
                SerialBT.println(message);
            #endif
        }
    }
    dual_wheel.stop();
}

String command_0 = "";
String command_1 = "";
String command_BT = "";
void loop() {
    // Check for serial commands
    if (Serial.available()) {
        char ch = Serial.read();
        if(ch == '\n'){
            processSerialCommand(command_0);
            command_0 = "";
        }else   command_0 += ch;
    }

    // // Check for serial commands
    // if (Serial1.available()) {

    //     char ch = Serial1.read();
    //     if(ch == '\n'){
    //         processSerialCommand(command_1);
    //         command_1 = "";
    //     }else   command_1 += ch;
    // }

    // Check for serial commands
    if (SerialBT.available()) {
        char ch = SerialBT.read();
        if(ch == '\n'){
            processSerialCommand(command_BT);
            command_BT = "";
        }else   command_BT += ch;
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

        // Serial.print(digitalRead(34));
        // Serial.print('\t');
        // Serial.print(digitalRead(34));
        // Serial.print('\t');
        // Serial.println(wheel_encoder.getCount());
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
        dual_wheel.set_speed(wheel_speed, wheel_speed, 60);
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

//suck process
void take_left(){
    relay_array.set_status(pump_left, 1);
    // relay_array.set_status(coil_left, 0);
}

void drop_left(){
    relay_array.set_status(pump_left, 0);
    // relay_array.set_status(coil_left, 1);
}

void take_right(){
    relay_array.set_status(pump_right, 1);
    // relay_array.set_status(coil_right, 0);
}

void drop_right(){
    relay_array.set_status(pump_right, 0);
    // relay_array.set_status(coil_right, 1);
}

void fan_left_(int stt) {
    relay_array.set_status(fan_left, stt);
}

void fan_right_(int stt) {
    relay_array.set_status(fan_right, stt);
}

void pile(int stt) {
    relay_array.set_status(xilanh, stt);
}
// double suck 

//hand and body process
#define body_speed 255
#define hand_speed 255
#define height_body 500
#define height_hand 250
int current_pos_hand = 0;
int current_pos_body = 0;

void body_process(char ch) {
    if(servo_stt) {

        int delta = 0;
        if(ch == '+')   delta = 5;
        if(ch == '-')   delta = -5;
        if(ch == '*')   delta = 50;
        if(ch == '/')   delta = -50;

        current_pos_body = constrain(current_pos_body+delta, 0, height_body);
        Serial1.println("T" + String(int(current_pos_body)));

    } else {
        if(ch == '0')   Serial1.println("B0");
        if(ch == '+')   Serial1.println("B" + String(int(body_speed/4)));
        if(ch == '-')   Serial1.println("B-" + String(int(body_speed/4)));
        if(ch == '*')   Serial1.println("B" + String(body_speed));
        if(ch == '/')   Serial1.println("B-" + String(body_speed));
    }
}

void hand_process(char ch) {
    if(servo_stt) {
        int delta = 0;
        if(ch == '+')   delta = 5;
        if(ch == '-')   delta = -5;
        if(ch == '*')   delta = 50;
        if(ch == '/')   delta = -50;

        current_pos_hand = constrain(current_pos_hand+delta, 0, height_hand);
        hand_servo.goto_position_mm(current_pos_hand);
    } else {
        if(ch == '0')   hand_motor.setSpeed(0);
        if(ch == '+')   hand_motor.setSpeed(hand_speed/4);
        if(ch == '-')   hand_motor.setSpeed(-hand_speed/4);
        if(ch == '*')   hand_motor.setSpeed(hand_speed);
        if(ch == '/')   hand_motor.setSpeed(-hand_speed);
    }
}


//combo process


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
    if(robot_type == ROBOT_NAME_1) {
        //bật hút
        fan_left_(1);
        fan_right_(1);
        //hạ tay 
        hand_servo.goto_position_mm(0);
        //nâng tay
        hand_servo.goto_position_mm(200);

    } else if (robot_type == ROBOT_NAME_2) {
        //bật hút
        fan_left_(1);
        fan_right_(1);
        //hạ tay 
        hand_servo.goto_position_mm(0);
        //nâng tay
        hand_servo.goto_position_mm(200);

    } else if (robot_type == ROBOT_NAME_3) {
        //bật hút
        fan_left_(1);
        fan_right_(1);
        //hạ tay 
        hand_servo.goto_position_mm(0);
        //nâng tay
        hand_servo.goto_position_mm(200);

    }
}
//combo 3 function: drop ball automatic
void drop_ball() {
    // if(robot_type == ROBOT_NAME_1) {
    //     //act 1: hạ tay
    //     body_servo.goto_position_mm( );
    //     //act 2: lui
        
        
    // } else if (robot_type == ROBOT_NAME_2) {
    //     //act 1: hạ tay
    //     body_servo.goto_position_mm(int );
    //     //act 2: lui
    //     dual_wheel.set_speed(-100);
    //     dual_wheel.set_speed(0);
    //     dual_wheel.set_speed(0);
    // } else if (robot_type == ROBOT_NAME_3) {
    //     //act 1: hạ tay
    //     body_servo.goto_position_mm(int );
    //     //act 2: lui
        
    // }
}
//combo 4 function: hút quân lương bán tự động 
void suck_box() {
    // if(robot_type == ROBOT_NAME_1) {
    //     //act 1: bật hút
    //     take_left();
        
    //     //act 2: hạ tay xuống hết + 
    //     body_servo.goto_position_mm(int min_body);
    //     //act 3: dài tay sau 
    //     hand_servo.goto_position_mm(int max_hand);
    //     //act 4: thu tay sau
    //     hand_servo.goto_position_mm(int );
    //     //act 5: nâng body
    //     body_servo.goto_position_mm(int );
        
    // } else if (robot_type == ROBOT_NAME_2) {
    //     //act 1: bật hút
    //     relay_array.set_status(coil_left, 0);
    //     relay_array.set_status(pump_left, 1);
    //     //act 2: hạ tay xuống hết + 
    //     body_servo.goto_position_mm(int min_body);
    //     //act 3: dài tay sau 
    //     hand_servo.goto_position_mm(int max_hand);
    //     //act 4: thu tay sau
    //     hand_servo.goto_position_mm(int );
    //     //act 5: nâng body
    //     body_servo.goto_position_mm(int );
        
    // } else if (robot_type == ROBOT_NAME_3) {
    //     //act 1: bật hút
    //     relay_array.set_status(coil_left, 0);
    //     relay_array.set_status(pump_left, 1);
    //     //act 2: hạ tay xuống hết + 
    //     body_servo.goto_position_mm(int min_body);
    //     //act 3: dài tay sau 
    //     hand_servo.goto_position_mm(int max_hand);
    //     //act 4: thu tay sau
    //     hand_servo.goto_position_mm(int );
    //     //act 5: nâng body
    //     body_servo.goto_position_mm(int );
        
    // }
}

//combo 5 function: kẹp cọc + thả cọc 
void pile_clamp() {
    // if(robot_type == ROBOT_NAME_1) {
    //     //act 1: kẹp xilanh
    //     relay_array.set_status(xilanh, 1);
    //     //act 2: keep forward
    //     dual_wheel.keep_forward(  );
    //     //act 3: thả xilanh
    //     relay_array.set_status(xilanh, 0);
    //     //act 4: tự động lui
    //     dual_wheel.keep_forward(- );
        
    // } else if (robot_type == ROBOT_NAME_2) {
    //     //act 1: kẹp xilanh
    //     relay_array.set_status(xilanh, 1);
    //     //act 2: keep forward
    //     dual_wheel.keep_forward(  );
    //     //act 3: thả xilanh
    //     relay_array.set_status(xilanh, 0);
    //     //act 4: tự động lui
    //     dual_wheel.keep_forward(- );
        
    // } else if (robot_type == ROBOT_NAME_3) {
    //     //act 1: kẹp xilanh
    //     relay_array.set_status(xilanh, 1);
    //     //act 2: keep forward
    //     dual_wheel.keep_forward(  );
    //     //act 3: thả xilanh
    //     relay_array.set_status(xilanh, 0);
    //     //act 4: tự động lui
    //     dual_wheel.keep_forward(- );
        
    // }
}

//combo 6 function: 


void update_k_PID(String command){
    float value = command.substring(1).toFloat();

    // if (command.startsWith("P")) {
    //     forward_pid.set_P(value);
    // }
    // if (command.startsWith("I")) {
    //     forward_pid.set_I(value);
    // }
    // if (command.startsWith("D")) {
    //     forward_pid.set_D(value);
    // }

    if (command.startsWith("P")) {
        rotate_pid.set_P(value);
    }
    if (command.startsWith("I")) {
        rotate_pid.set_I(value);
    }
    if (command.startsWith("D")) {
        rotate_pid.set_D(value);
    }
}

void process_combo(int value){
    if(value == 1)  combo_1();
    if(value == 23)  auto_forward(1200);
}

void processSerialCommand(String command) {
    // Serial.println(command);
    command.trim();  // Remove any leading/trailing whitespace

    if(command.startsWith("E")) {
        if(command.substring(1).toInt()) servo_stt = true;
        else servo_stt = false;
    }

    if(command.startsWith("B")) {
        char ch = command.charAt(1);
        body_process(ch);
    }

    if (command.startsWith("H")) {
        char ch = command.charAt(1);
        hand_process(ch);
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
    

    //Relay 
    if (command.startsWith("TL")) {
        int value = command.substring(2).toInt();
        if(value == 0)  drop_left();
        else            take_left();
    }
    
    if (command.startsWith("TR")) {
        int value = command.substring(2).toInt();
        if(value == 0)  drop_right();
        else            take_right();
    }
    if (command.startsWith("TD")) {
        int value = command.substring(2).toInt();
        if(value == 0) {
            drop_right();
            drop_left();
        }  
        else {
            take_right();
            take_left();
        }
    }
    
    if (command.startsWith("XL")) {
        int value = command.substring(2).toInt();
        fan_left_(value);
    }
    
    if (command.startsWith("XR")) {
        int value = command.substring(2).toInt();
        fan_right_(value);
    }
    
    if (command.startsWith("XD")) {
        int value = command.substring(2).toInt();
        fan_right_(value);
        fan_left_(value);
    }

    if (command.startsWith("XP")) {
        int value = command.substring(2).toInt();
        pile(value);
    }
        
    //set yard
    if(command.startsWith("Y")) {
        if(command.charAt(1) == 'L') yard = left_mode;
        if(command.charAt(1) == 'R') yard = right_mode;
    }

    if (command.startsWith("C")) {
        int value = command.substring(1).toInt();
        process_combo(value);
    }

    if (command.startsWith("A")) {
        angle = command.substring(1).toInt()-20;
    }
    
    // if (command.startsWith("D")) {
    //     duration = command.substring(1).toInt();
    // }
    
    if (command.startsWith("R")) {
        if(command.charAt(1) == 'H') {
            hand_servo.reset(command.substring(2).toInt());
        }
        if(command.charAt(1) == 'B') {
            Serial1.println("R" + command.substring(2));
        }
    }

    if (command.startsWith("W")) {
        int angle = command.substring(1).toInt();
        rotate_CCW(current_dir + angle*10);
    }


}