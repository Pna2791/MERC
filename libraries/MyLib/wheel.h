#ifndef WHEEL_H
#define WHEEL_H



#define FW_SPEED    180
#define TURN_SPEED  255


class Motor {
    private:
        int dir_pin, speed_pin;
    
    public:
        Motor(int dir_pin, int speed_pin){
            this->dir_pin = dir_pin;
            this->speed_pin = speed_pin;

            pinMode(dir_pin, OUTPUT);
            pinMode(speed_pin, OUTPUT);
        }
    
        void setSpeed(int val) {
            if (val == 0) {
                digitalWrite(speed_pin, 0);
                digitalWrite(dir_pin, 0);
                return;
            }

            if (val > 0) {
                digitalWrite(dir_pin, 0);
            } else {
                digitalWrite(dir_pin, 1);
            }

            if (abs(val) == 255) {
                digitalWrite(speed_pin, 1);
            } else {
                analogWrite(speed_pin, abs(val));
            }
        }

        void stop(){
            setSpeed(0);
        }
};
    

class Chassis {
    private:
        Motor &left_motor;
        Motor &right_motor;

    public: 
        Chassis(Motor &left_motor, Motor &right_motor) {
            this->left_motor = left_motor;
            this->right_motor = right_motor; 
        }

        void move_speed(int val){
            left_motor.setSpeed(val);
            right_motor.setSpeed(val);
        }

        void rotate_CCW(int val){
            left_motor.setSpeed(-val);
            right_motor.setSpeed(val);
        }

        void stop(){
            move_speed(0);
        }

        void move_forward(int delta, int speed=FW_SPEED){
            if(delta == 0){
                left_motor.setSpeed(speed);
                right_motor.setSpeed(speed);
            }else if(delta > 0){
                left_motor.setSpeed(speed-delta);
                right_motor.setSpeed(speed);
            }else{
                left_motor.setSpeed(speed);
                right_motor.setSpeed(speed+delta);
            }
        }

};


#endif