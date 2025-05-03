#include <Encoder.h>
#include <Motor.h>
#include <vector>
using namespace std;

Encoder wheel_encoder(34, 35);
Motor motor_right = Motor(19, 22);

int values[256];
void dyno(){
	for(int i=0; i<256; i++){
		motor_right.setSpeed(-i);
		delay(500);
		long current = wheel_encoder.getCount();
		delay(3000);
		current = wheel_encoder.getCount() - current;
		values[i] = current;
		Serial.println(String(-i) + "\t" + String(current));
	}
}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	wheel_encoder.begin();
    delay(1000);

	dyno();
}
int cnt = 0;
void loop() {

}
