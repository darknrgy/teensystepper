#include "travel.h"
#include "stepper.h"

#define MOTOR_COUNT 2
#define PIN_X_STEP 13
#define PIN_X_DIR 14
#define PIN_Y_STEP 15
#define PIN_Y_DIR 16


IntervalTimer timer;

unsigned long elapsed_u_sec = 0;

struct Motors {
	TravelItem travel_item;
	Travel travel;
	Stepper stepper;
} motors[MOTOR_COUNT];


void set_gimbal(int x, int y) {
	static float t = (float) elapsed_u_sec / 1000000;

	noInterrupts();
	travel_create(&motors[0].travel, &motors[0].travel_item, MAX_V, MAX_A, t, x);
	travel_create(&motors[1].travel, &motors[1].travel_item, MAX_V, MAX_A, t, y);
	interrupts();
}

void setup() {
	Serial.begin(9600);

	motors[0].stepper.step_pin = PIN_X_STEP;
	motors[0].stepper.dir_pin = PIN_X_DIR;
	motors[1].stepper.step_pin = PIN_Y_STEP;
	motors[1].stepper.dir_pin = PIN_Y_DIR;

	pinMode(PIN_X_STEP, OUTPUT);
	pinMode(PIN_X_DIR, OUTPUT);
	pinMode(PIN_Y_STEP, OUTPUT);
	pinMode(PIN_Y_DIR, OUTPUT);

	timer.begin(tick, TICK_INTERVAL);  // 50 = 100us, or 10kHz
}

void tick() {
	static volatile float elapsed_sec;
	static volatile int i;
	static volatile bool can_reset_time;

	noInterrupts();
	can_reset_time = true;
	elapsed_u_sec += TICK_INTERVAL;
	elapsed_sec = (float) elapsed_u_sec / 1000000;
	for (i = 0; i < MOTOR_COUNT; i++) {
		travel_tick(&motors[i].travel, &motors[i].travel_item, elapsed_sec);
  		motors[i].stepper.p = motors[i].travel_item.p;
		stepper_step(&motors[i].stepper, elapsed_u_sec);	
		if (motors[i].travel.enable) {
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
