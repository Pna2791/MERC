#ifndef ENCODER_TIMER_H
#define ENCODER_TIMER_H


#include <esp_timer.h>


class EncoderTimer {
public:
    EncoderTimer(int interruptPin, int directionPin, int timerId, uint16_t intervalMicros=1000)
        : interruptPin(interruptPin), directionPin(directionPin), timerId(timerId), intervalMicros(intervalMicros) {}

    void begin() {
        pinMode(interruptPin, INPUT_PULLUP);
        pinMode(directionPin, INPUT_PULLUP);

        const esp_timer_create_args_t timer_args = {
            .callback = &EncoderTimer::onTimer,
            .arg = this,
            .name = "encoder_timer"
        };

        esp_timer_create(&timer_args, &espTimer);
        esp_timer_start_periodic(espTimer, intervalMicros);
    }

    // Define your encoder logic here
    void process_wheel() {
        if(pre_stt != digitalRead(interruptPin)){
            if(pre_stt) encoderCount += !digitalRead(directionPin) ? 1 : -1;
            pre_stt = !pre_stt;
        }
    }

    long getCount() {
        return encoderCount;
    }

private:
    bool pre_stt = false;
    volatile long encoderCount = 0;
    int interruptPin;
    int directionPin;
    int timerId;
    uint16_t intervalMicros;
    esp_timer_handle_t espTimer;

    static void onTimer(void* arg) {
        static_cast<EncoderTimer*>(arg)->process_wheel();
    }
};


#endif