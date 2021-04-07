#define STEPPER_TYPE_MONOPOLE
// #define STEPPER_TYPE_STEP_DIR

#ifdef STEPPER_TYPE_STEP_DIR
#include "stepper.h"
#endif
#ifdef STEPPER_TYPE_MONOPOLE
#include "monostepper.h"
#endif


#include "travel.h"


#define MOTOR_COUNT 2
#define PIN_X_STEP 13
#define PIN_X_DIR 14
#define PIN_Y_STEP 15
#define PIN_Y_DIR 16


IntervalTimer timer;

unsigned long elapsed_u_sec = 0;

Dimension dims[2];

void set_gimbal(int x, int y) {
	static float t;
	t = (float) elapsed_u_sec / 1000000;

	noInterrupts();
	dims[0].solve_for_min_time(MAX_V, MAX_A, t, x);
	dims[1].solve_for_min_time(MAX_V, MAX_A, t, y);

	if (!dims[0].travel.retrigger && !dims[0].travel.retrigger) {		
		if (dims[0].get_travel_time() > dims[1].get_travel_time()) {
			dims[1].solve_for_complement(MAX_V, MAX_A, t, y, dims[0]);
		} else {
			dims[0].solve_for_complement(MAX_V, MAX_A, t, x, dims[1]);
		}
	}

	interrupts();
}

void setup() {
	Serial.begin(9600);

	pinMode(13, OUTPUT);
	pinMode(14, OUTPUT);
	pinMode(15, OUTPUT);
	pinMode(16, OUTPUT);
	pinMode(17, OUTPUT);
	pinMode(18, OUTPUT);
	pinMode(19, OUTPUT);
	pinMode(20, OUTPUT);

	#ifdef STEPPER_TYPE_MONOPOLE
	stepper_set_pins(&dims[0].stepper, 13, 14, 15, 16);
	stepper_set_pins(&dims[1].stepper, 17, 18, 19, 20);
	#endif

	timer.begin(tick, TICK_INTERVAL);  // 50 = 100us, or 10kHz
}

void tick() {
	
	static volatile int i;
	static volatile bool can_reset_time;

	noInterrupts();

	elapsed_u_sec += TICK_INTERVAL;
	
	can_reset_time = true;
	for (i = 0; i < MOTOR_COUNT; i++) {
		dims[i].tick(elapsed_u_sec);
		if (dims[i].is_enabled()) {
			can_reset_time = false;
		}

	}

	if (can_reset_time) elapsed_u_sec = 0;

	interrupts();
}

/*void loop() {
	float x, y;
	x = -1000;
	y = 100;
	set_gimbal(x, y);
	delay(999999999);

}*/



void loop() {
	float x, y;

	while(true) {
		x = random(-250,250);
		y = random(-250,250);
		set_gimbal(x, y);
		delay(random(100, 1000));
	}
	
}
