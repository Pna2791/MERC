#ifndef DC_SERVO_H
#define DC_SERVO_H

class DC_Servo {
    private: 
        int plush_per_mm, target_position, current_position;
    public:
        DC_Servo() {}
        void goto_by_mm(int x) {
            goto_position(x*plush_per_mm);    
        }

        void goto_position(int value){
            // Set target position

            run();
        }
        int get_target_mm() {
            return target_position / plush_per_mm;
        }
        void run(){

        }
        void reset_position(int offset = 0) {
            target_position  = offset * plush_per_mm;
            current_position = target_position;
            stop();
        }
        void stop() {

        }
};


#endif