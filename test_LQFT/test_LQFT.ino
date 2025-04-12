#define speed_left_pin  6
#define speed_right_pin 11

void setup() {
    pinMode(speed_left_pin, OUTPUT);
    pinMode(speed_right_pin, OUTPUT);

}

void loop() {
    digitalWrite(speed_left_pin, 1);
    digitalWrite(speed_right_pin, 1);
    delay(2000);

    digitalWrite(speed_left_pin, 0);
    digitalWrite(speed_right_pin, 0);
    delay(2000);
}
