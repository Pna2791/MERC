#ifndef ENCODER_H
#define ENCODER_H

#define encoder_A       2  // Should be interrupt-capable pin
#define plush_per_cm    2
#define ENCODER_OFFSET  3

volatile long encoderCount = 0;


void setup_encoder(){
    pinMode(encoder_A, INPUT_PULLUP);

    noInterrupts();  // Disable interrupts during setup

    TCCR2A = 0;      // Clear Timer/Counter Control Registers
    TCCR2B = 0;
    TCNT2 = 0;       // Reset counter

    // 16 MHz / 128 prescaler = 125,000 ticks/sec
    // 6.25 ms = 0.00625 sec → 0.00625 * 125,000 = 781.25 ticks
    OCR2A = 781;     // Set compare match value

    TCCR2A |= (1 << WGM21);                     // CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS20);        // Prescaler = 128
    TIMSK2 |= (1 << OCIE2A);                    // Enable Timer2 Compare Match A interrupt

    interrupts();    // Enable interrupts
}

ISR(TIMER2_COMPA_vect) {
    static bool previous_stt = 0;
    bool stt = digitalRead(encoder_A);
    if(stt != previous_stt){
        previous_stt = stt;
        encoderCount++;
    }
}


#endif
