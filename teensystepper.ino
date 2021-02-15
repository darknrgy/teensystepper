#include "travel.h"
#include "stepper.h"

IntervalTimer timer;
Stepper stepper1;
Travel travel1;
TravelItem travel_item1;
unsigned long elapsed_u_sec = 0;

void setup() {
	Serial.begin(9600);
	pinMode(13, OUTPUT);
	pinMode(14, OUTPUT);
	timer.begin(tick, TICK_INTERVAL);  // 50 = 100us, or 10kHz
}

void tick() {
	static float elapsed_sec;

	noInterrupts();
	elapsed_u_sec += TICK_INTERVAL;
	elapsed_sec = (float) elapsed_u_sec / 1000000;  	
  	travel_tick(&travel1, &travel_item1, elapsed_sec);
  	stepper1.p = travel_item1.p;
	stepper_step(&stepper1, elapsed_u_sec);
	if (!travel1.enable) elapsed_u_sec = 0;
	interrupts();
}

void loop() {
	float p = 0;
	float elapsed_sec;

	while(true) {

		///////////////
		noInterrupts();
		p = random(-3200,3200);
		elapsed_sec = (float) elapsed_u_sec / 1000000;
		travel_create(&travel1, &travel_item1, MAX_V, MAX_A, elapsed_sec, p);
		Serial.println(p);
		interrupts();
		///////////////

		delay(random(100,400));
	}
	
}
