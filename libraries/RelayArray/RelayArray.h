#ifndef RELAY_ARRAY_H
#define RELAY_ARRAY_H


class Relay_Array{
    private:
        bool gate_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        int pin_index[8] = {0, 1, 2, 3, 4, 5, 6, 7};

        int latchPin, clockPin, dataPin;

    public:
        Relay_Array(int latchPin, int clockPin, int dataPin){
            this->latchPin  = latchPin;
            this->clockPin  = clockPin;
            this->dataPin   = dataPin;

            pinMode(latchPin, OUTPUT);
            pinMode(clockPin, OUTPUT);
            pinMode(dataPin, OUTPUT);
        }
        void udpate_status(){
            byte numberToDisplay = 0;

            for(int i=0; i<8; i++)
                if(gate_status[pin_index[i]])
                    numberToDisplay |= 1 << i;

            digitalWrite(latchPin, 0);
            shiftOut(dataPin, clockPin, MSBFIRST, numberToDisplay);
            digitalWrite(latchPin, 1);
        }

        void set_status(int gate, bool status){
            gate_status[gate] = status;
            udpate_status();
        }

        void reset(){
            for(int i=0; i<8; i++)
                gate_status[i] = 0;
            
            udpate_status();
        }
};


#endif