#ifndef WHEEL_H
#define WHEEL_H


#include <PID_Control.h>

#include "BluetoothSerial.h"

class Motor {
    private:
        int dir_pin, speed_pin, pre_speed;
    
    public:
        Motor(int dir_pin, int speed_pin){
            this->dir_pin   = dir_pin;
            this->speed_pin = speed_pin;

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
            analogWrite(speed_pin, abs(val));
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
            if(target_dir > 1350 && direction < -450)       direction += 3600;
            else if(target_dir > 450 && direction < -1350)  direction += 3600;

            if(target_dir < -450 && direction > 450)
                direction -= 3600;

            float delta_value = forward_pid.compute(target_dir, direction)/255;
            serialPort.print(delta_value);
            serialPort.print(' ');
            serialPort.println(target_dir-direction);
            move_forward(delta_value, speed);
        }
};


#endif