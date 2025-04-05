#ifndef WHEEL_H
#define WHEEL_H

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
            static int pre_speed = 0;
            if (val == 0) {
                if(pre_speed < 0)   digitalWrite(dir_pin, 0);
                else                digitalWrite(dir_pin, 1);
                analogWrite(speed_pin, 0);
                return;
            }

            if (val > 0) {
                digitalWrite(dir_pin, 0);
            } else {
                digitalWrite(dir_pin, 1);
            }

            if (abs(val) == 255) {
                analogWrite(speed_pin, 255);
            } else {
                analogWrite(speed_pin, abs(val));
            }

            pre_speed = val;
        }

        void stop(){
            setSpeed(0);
        }
};

#endif