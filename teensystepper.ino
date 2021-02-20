#include "stepper.h"
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
	static float t = (float) elapsed_u_sec / 1000000;

	noInterrupts();
	dims[0].solve_for_min_time(MAX_V, MAX_A, t, x);
	dims[1].solve_for_min_time(MAX_V, MAX_A, t, y);
	interrupts();
}

void setup() {
	Serial.begin(9600);

	pinMode(PIN_X_STEP, OUTPUT);
	pinMode(PIN_X_DIR, OUTPUT);
	pinMode(PIN_Y_STEP, OUTPUT);
	pinMode(PIN_Y_DIR, OUTPUT);

	dims[0].set_stepper_pins(PIN_X_STEP, PIN_X_DIR);
	dims[1].set_stepper_pins(PIN_Y_STEP, PIN_Y_DIR);

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

void loop() {
	float x, y;

	while(true) {
		x = random(-500,500);
		y = random(-500,500);
		set_gimbal(x, y);
		delay(random(1000, 1000));
	}
	
}
