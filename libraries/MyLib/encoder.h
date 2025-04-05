#ifndef ENCODER_H
#define ENCODER_H



class Encoder {
    private:
        int pin_A, pin_B;
        long counter;
    
    public:
        Encoder(int pin_A, int pin_B){
            this->pin_A = pin_A;
            this->pin_B = pin_B;

            pinMode(pin_A, INPUT_PULLUP);
            pinMode(pin_B, INPUT_PULLUP);

            attachInterruptEx(pin_A, [this] { ISR(); }, RISING);
        }

        void ISR() {
            if(digitalRead(pin_B))  counter++;
            else                    counter--;
        }

        long get_position(){
            return counter;
        }
};

#endif
