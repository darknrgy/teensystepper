#define TICK_INTERVAL 10
#define STEP_TIME 1000
#define STARTING_STATE 0b0001
#define STEP_WAIT 1
#define STEP_READY 2

struct Stepper {
	int pin_a, pin_b, pin_c, pin_d;

	volatile float p = 0;
	volatile long real_p = 0;
	volatile int state = STEP_READY;
	volatile int pin_state = STARTING_STATE;

	elapsedMicros elapsed = 0;
};

int stepper_shift(int state, int dir) {
	if (dir > 0) {
		state = state >> 0b1;
	} else {
		state = state << 0b1;
	}

	if (state < 0b0001) state = 0b1000;
	if (state > 0b1000) state = 0b0001;

	return state;
}

void stepper_step(Stepper *stepper, long elapsed) {
	// do not allocate any memory
	static long p; 
	p = stepper->p;

	if (stepper->state == STEP_WAIT) {
		// minimum step time
		if (stepper->elapsed > STEP_TIME) {
			stepper->state = STEP_READY;
			stepper->elapsed = 0;
		}
	} 

	if (stepper->state == STEP_READY) {
		// step forward
		if (p > stepper->real_p) {
			stepper->real_p ++;
			stepper->pin_state = stepper_shift(stepper->pin_state, 1);
			stepper->state = STEP_WAIT;
			stepper->elapsed = 0;
		} 

		// step backward
		else if (p < stepper->real_p) {
			stepper->real_p --;
			stepper->pin_state = stepper_shift(stepper->pin_state, -1);
			stepper->state = STEP_WAIT;
			stepper->elapsed = 0;	
		}
	}

	digitalWrite(stepper->pin_a, stepper->pin_state & 0b1000);
	digitalWrite(stepper->pin_b, stepper->pin_state & 0b0100);
	digitalWrite(stepper->pin_c, stepper->pin_state & 0b0010);
	digitalWrite(stepper->pin_d, stepper->pin_state & 0b0001);

}

void stepper_set_pins(Stepper *stepper, int a, int b, int c, int d) {
	stepper->pin_a = a;
	stepper->pin_b = b;
	stepper->pin_c = c;
	stepper->pin_d = d;
}