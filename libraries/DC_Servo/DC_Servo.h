#ifndef DC_SERVO_H
#define DC_SERVO_H

#define DEBUG
// #define BLDC


class DC_servo{
    private:
        Motor &motor;
        #ifndef BLDC
            EncoderTimer &encoder;
        #else
            Encoder &encoder;
        #endif


        PIDController &pid_controller;
        float pluss_per_mm;

        int target_pos, offset;
    
    public:

        #ifndef BLDC
            DC_servo(Motor &motor, EncoderTimer &encoder, PIDController &pid_controller, float plush_per_mm=50) 
                : motor(motor), encoder(encoder), pid_controller(pid_controller){
                    this->pluss_per_mm = plush_per_mm;
                }
        #else
            DC_servo(Motor &motor, Encoder &encoder, PIDController &pid_controller, float plush_per_mm=50) 
                : motor(motor), encoder(encoder), pid_controller(pid_controller){
                    this->pluss_per_mm = plush_per_mm;
                }
        #endif
        
        void goto_position_mm(int target){
            goto_position(pluss_per_mm*target);
        }

        void goto_position(int target){
            this->target_pos = offset+target;
            run();
        }
        
        void reset(int value=0){
            int current_pos = encoder.getCount();
            offset = current_pos - pluss_per_mm*value;
            target_pos = current_pos;
        }

        void stop(){
            motor.stop();
        }

        void run(){
            int current_pos = encoder.getCount();
            int value = pid_controller.compute(target_pos, current_pos);

            #ifdef DEBUG
                Serial.print(current_pos);
                Serial.print('\t');
                Serial.println(value);
            #endif

            motor.setSpeed(value);
        }
};


#endif