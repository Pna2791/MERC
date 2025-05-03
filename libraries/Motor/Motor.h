#ifndef WHEEL_H
#define WHEEL_H


#include <PID_Control.h>
#define FW_SPEED    255
#define TURN_SPEED  255
#include "BluetoothSerial.h"


// #define DEBUG
#define LINEAR_MAPPER


#ifdef LOG_MAPPER
    const uint8_t log_mapper[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6,
    6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8,
    8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12,
    12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17,
    17, 17, 18, 18, 19, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23,
    24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32,
    32, 33, 34, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43,
    44, 45, 46, 47, 48, 49, 49, 50, 51, 52, 53, 54, 55, 56, 58, 59,
    60, 61, 62, 63, 64, 66, 67, 68, 69, 71, 72, 73, 75, 76, 77, 79,
    81, 82, 84, 85, 87, 88, 90, 92, 93, 95, 97, 99, 101, 102, 104, 106,
    108, 110, 112, 114, 116, 118, 121, 123, 125, 128, 130, 132, 135, 137, 140, 142,
    145, 148, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 181, 184, 187, 191,
    194, 198, 202, 205, 209, 213, 217, 221, 225, 229, 233, 238, 242, 246, 250, 255
    };
#endif

#ifdef LINEAR_MAPPER
    const uint8_t mapper[511] = {
        237, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226, 225, 224, 223, 222, 221, 220, 219, 218, 217, 216, 215, 213, 212, 211, 210, 209, 208, 207, 206, 205, 204, 203, 202, 201, 200, 199, 198, 197, 196, 195, 194, 193, 192, 191, 190, 189, 188, 187, 186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173, 172, 171, 170, 169, 167, 166, 165, 164, 163, 162, 161, 160, 159, 158, 157, 156, 155, 154, 153, 152, 151, 150, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 21, 22, 22, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 58, 58, 59, 60, 61, 62, 63, 64, 64, 65, 66, 67, 68, 69, 70, 70, 71, 72, 73, 74, 75, 76, 76, 77, 78, 79, 80, 81, 81, 82, 83, 84, 85, 86, 87, 87, 88, 89, 90, 91, 92, 93, 93, 94, 95, 96, 97, 98, 99, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110, 111, 111, 112, 113, 114, 115, 116, 117, 117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 126, 127, 128, 129, 129, 130, 131, 132, 133, 134, 135, 135, 136, 137, 138, 139, 140, 141, 141, 142, 143, 144, 145, 146, 147, 147, 148, 149, 150, 151, 152, 153, 153, 154, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 164, 164, 165, 166, 167, 168, 169, 170, 170, 171, 172, 173, 174, 175, 176, 176, 177, 178, 179, 180, 181, 182, 182, 183, 184, 185, 186, 187, 188, 188, 189, 190, 191, 192, 193, 194, 194, 195, 196, 197, 198, 199, 200, 200, 201, 202, 203
    };
#endif


class Motor {
    private:
        int dir_pin, speed_pin, pre_speed, direction;
    
    public:
        Motor(int dir_pin, int speed_pin, int direction=0){
            this->dir_pin   = dir_pin;
            this->speed_pin = speed_pin;
            this->direction = direction;

            pinMode(dir_pin, OUTPUT);
            pinMode(speed_pin, OUTPUT);
        }
    
        void setSpeed(int val) {
            if (val == 0) {
                digitalWrite(dir_pin, pre_speed>0);
                analogWrite(speed_pin, 0);
                return;
            }

            digitalWrite(dir_pin, val<0);
            if(direction == 0)  analogWrite(speed_pin, abs(val));
            else                analogWrite(speed_pin, mapper[255+val*direction]);
            pre_speed = val;
        }

        void stop(){
            setSpeed(0);
        }
};


class Chassis {
    private:
        Motor &left_motor;
        Motor &right_motor;
        PIDController &forward_pid;

        int left_speed, right_speed;
        long status_index = 0;
        int soft_start_duration = 1;
        int chassis_speed = 0;
        int target_dir = 0;


    public: 
        int code = 0;

        Chassis(Motor &left_motor, Motor &right_motor, PIDController &forward_pid) 
            : left_motor(left_motor), right_motor(right_motor), forward_pid(forward_pid){}

        void move_speed(int val){
            left_motor.setSpeed(val);
            right_motor.setSpeed(val);
        }

        void rotate_CCW(int val){
            left_motor.setSpeed(-val);
            right_motor.setSpeed(val);
        }

        void stop(){
            left_speed = 0;
            right_speed = 0;
            code = 0;
            move_speed(0);
        }

        void move_forward(float delta, int speed=0){
            if(delta == 0){
                left_speed = speed;
                right_speed = speed;
            }else if(delta > 0){
                left_speed = speed-delta*speed;
                right_speed = speed;
            }else{
                left_speed = speed;
                right_speed = speed+delta*speed;
            }
        }
        
        void set_speed(int left, int right, int softstart=1){
            left_speed = left;
            right_speed = right;
            soft_start_duration = softstart;
            status_index = 0;
        }

        void set_target_dir(int dir){
            target_dir = dir;
        }

        void run(){
            status_index++;
            if(status_index >= soft_start_duration){
                left_motor.setSpeed(left_speed);
                right_motor.setSpeed(right_speed);
                return;
            }else{
                int offset = 5;
                float k = float(status_index+offset) / (soft_start_duration+offset);
                left_motor.setSpeed(left_speed*k);
                right_motor.setSpeed(right_speed*k);
            }
        }

        void keep_forward(int direction, int speed, BluetoothSerial &serialPort){
        // void keep_forward(int direction, int speed){
            if(target_dir > 1350 && direction < -450)       direction += 3600;
            else if(target_dir > 450 && direction < -1350)  direction += 3600;

            if(target_dir < -450 && direction > 450)
                direction -= 3600;

            float delta_value = forward_pid.compute(target_dir, direction)/255;
            #ifdef DEBUG
                String message = String(delta_value) + '\t' + String(target_dir-direction);
                serialPort.println(message);
            #endif
            move_forward(delta_value, speed);
        }
};


#endif