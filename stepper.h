#define TICK_INTERVAL 1
#define STEP_ON_TIME 1
#define STEP_TIME 2
#define STEP_OFF_TIME (STEP_TIME - STEP_ON_TIME)
#define STEP_OFF 0
#define STEP_ON 1
#define STEP_WAIT 2
#define DIR_PIN_POS 1
#define DIR_PIN_NEG 0

struct Stepper {
	int step_pin = 13;
	int dir_pin = 14;
	volatile float p = 0;
	volatile long real_p = 0;
	volatile int state = STEP_WAIT;
	volatile int dir_pin_state = STEP_OFF;
	elapsedMicros elapsed = 0;
};

void stepper_step(Stepper *stepper, long elapsed) {
	// do not allocate any memory
	static long p; p = (long) (stepper->p);
	static long step_pin_state;


	if (stepper->state == STEP_ON) {
		// minimum pulse width
		if (stepper->elapsed > STEP_ON_TIME) {
			stepper->state = STEP_OFF;
			stepper->elapsed = 0;
		}
	} 

	else if (stepper->state == STEP_OFF) {
		// minimum wavelength (maximum step frequency)
		if (stepper->elapsed > STEP_OFF_TIME) {
			stepper->state = STEP_WAIT;
			stepper->elapsed = 0;
		}
	} 

	if (stepper->state == STEP_WAIT) {
		// step forward
		if (p > stepper->real_p) {
			stepper->real_p ++;
			stepper->dir_pin_state = DIR_PIN_POS;
			stepper->state = STEP_ON;
			stepper->elapsed = 0;
		} 

		// step backward
		else if (p < stepper->real_p) {
			stepper->real_p --;
			stepper->dir_pin_state = DIR_PIN_NEG;
			stepper->state = STEP_ON;
			stepper->elapsed = 0;	
		}
	}

	step_pin_state = stepper->state == STEP_ON ? STEP_ON : STEP_OFF;
	digitalWrite(stepper->step_pin, step_pin_state);
  	digitalWrite(stepper->dir_pin, stepper->dir_pin_state);
} // step: 13, dir: 14