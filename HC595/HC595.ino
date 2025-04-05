int latchPin = 5;
int clockPin = 18;
int dataPin = 23;


void setup() {
  Serial.begin(115200);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

bool led_status[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
void show_leds() {
  byte numberToDisplay = 0;  // Initialize shift register data
  for (byte i = 0; i < 8; i++) {
    if (led_status[i]) {                  // If led_status[i] is 1, set the corresponding bit
      numberToDisplay |= (1 << (i + 1));  // Shift to correct LED position
    }
  }

  // Send data to shift register
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, ~numberToDisplay);
  digitalWrite(latchPin, HIGH);
}

void loop() {
  for (byte i = 0; i < 8; i++) {    // Loop through each LED position
    byte numberToDisplay = 1 << i;  // Shift 1 to the correct bit position

    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, numberToDisplay);
    digitalWrite(latchPin, HIGH);

    Serial.print("LED ");
    Serial.print(i);
    Serial.println(" ON");

    delay(2000);  // LED stays on for 2 seconds
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 0);  // Turn off all LEDs
    digitalWrite(latchPin, HIGH);

    Serial.println("All LEDs OFF");
    delay(2000);  // Sleep for 1 second
  }
}