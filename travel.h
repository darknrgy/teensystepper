#define MAX_V 8000
#define MAX_A 10000

/*
fastest tested
#define MAX_V 80000
#define MAX_A 1000000
*/

struct Travel {
	bool enable = false;
	float p_start = 0;
	float p_next = 0;
	bool retrigger = false;
	float t_start = 0;
	float d_dest = 0;
	float v0 = 0;
	float t_accel = 0;
	float t_cruise = 0;
	float t_decel = 0;
	float d_accel = 0;
	float d_cruise = 0;
	float maxv = 0;
	float maxa = 0;
	int dir =1;
};

struct Travel_Item {
	float p = 0; // position
	float v = 0; // velocity
};

class Dimension {
	public:
	Travel_Item travel_item;
	Travel travel;
	Stepper stepper;

	void set_stepper_pins(int step_pin, int dir_pin) {
		stepper.step_pin = step_pin;
		stepper.dir_pin = dir_pin;
	}

	void solve_for_min_time(float maxv, float maxa, float t_start, float p) {
		float v0 = travel_item.v;
		float d_dest = p - travel_item.p;

		float dd_0 = 0;
		float dd_accel = 0;
		float dd_cruise = 0;
		float dtotal = 0;
		float dt_0 = 0;
		float dt_reverse = 0;
		float dt_accel = 0;
		float dt_cruise = 0;
		float dt_decel = 0;
		float ttop = 0;
		float dtop = 0;

		if (v0 * d_dest > 0 && (abs(d_dest) < abs(0.5 * maxa * pow((v0 / maxa), 2)))) {
			// Impossible instruction to decel in time, queue up this instruction next
			travel.p_next = p;
			travel.retrigger = true;
			return;
		} else {
			travel.retrigger = false;
		}

		static int dir;

		travel.enable = true;
		travel.d_dest = d_dest;
		travel.p_start = travel_item.p;
		travel.t_start = t_start;
		travel.v0 = v0;
		travel.maxa = maxa;

		ttop = maxv / maxa;
		dtop = 0.5f * maxa * pow(ttop,2);

		if (d_dest > 0) dir = 1;
		else dir = -1;

		// calculate decel from v0 to 0
		dt_0 = v0 / maxa;
		dd_0 = 0.5f * maxa * pow(dt_0, 2);
		dtotal = abs(d_dest) + abs(dd_0);

		if (v0 * d_dest < 0) {
			// if going the wrong direction, add decel time to turn around
			dt_reverse = 2.0 * abs(dt_0);	
		} else {
			dt_reverse = 0;
		}

		if (dtop * 2 > dtotal) {
			// cruise speed not achieved, use midpoint to start decel
			dt_decel = sqrt(dtotal / maxa);
			maxv = maxa * dt_decel;
			dt_accel = (maxv - abs(v0)) / maxa;
			dd_accel = abs(v0) * dt_accel + 0.5f * maxa * pow(dt_accel, 2);
			dt_cruise = 0;
			dd_cruise = 0;
		} else {
			// cruise speed possible, create all 3 phases
			dt_accel = (maxv - abs(v0)) / maxa;
			dd_accel = abs(v0) * dt_accel + 0.5f * maxa * pow(dt_accel, 2);
			dt_decel = ttop;
			dd_cruise = abs(d_dest) - (dtop + dd_accel);
			dt_cruise = dd_cruise / maxv;
		}

		travel.t_accel = dt_reverse + dt_accel;
		travel.t_cruise = travel.t_accel + dt_cruise;
		travel.t_decel = travel.t_cruise + dt_decel;
		travel.d_accel = dd_accel;
		travel.d_cruise = travel.d_accel + dd_cruise;
		travel.maxv = maxv;
		travel.maxa = maxa;
		travel.dir = dir;
	}

	void tick(float elapsed_u_sec) {
		static volatile float elapsed_sec;
		elapsed_sec = (float) elapsed_u_sec / 1000000;
		solve_tick(elapsed_sec);
		stepper.p = travel_item.p;
		stepper_step(&stepper, elapsed_u_sec);	
	}

	void solve_tick(float t) {
		// do not allocate any memory
		static float dt;

		t = t - travel.t_start;

		if (!travel.enable){
			// disabled
			if (travel.retrigger) {
				travel.retrigger = false;
				solve_for_min_time(travel.maxv, travel.maxa, t, travel.p_next);
			}
		} else if (t > travel.t_decel) {
			// destination reached, disable
			travel.enable = false;
			travel_item.p = travel.p_start + travel.d_dest;
			travel_item.v = 0;
		} else if (t < travel.t_accel) {
			// accelerating
			travel_item.p = travel.p_start + travel.v0 * t + 0.5f * travel.maxa * travel.dir * pow(t, 2);
			travel_item.v = travel.v0 + travel.maxa * travel.dir * t;
		} else if (t < travel.t_cruise) {
			// cruising
			dt = t - travel.t_accel;
			travel_item.p = travel.p_start + travel.d_accel * travel.dir + travel.maxv * travel.dir * (dt);
			travel_item.v = travel.maxv * travel.dir;
		} else {
			// decelerating
			dt = t - travel.t_cruise;
			travel_item.p = travel.p_start + travel.d_cruise * travel.dir + travel.maxv * travel.dir * (dt) + 0.5f * (travel.maxa * (-travel.dir)) * pow(dt, 2);
			travel_item.v = travel.maxv * travel.dir - travel.maxa * travel.dir * dt;
		}
	}

	bool is_enabled() {
		return travel.enable;
	}

	float get_travel_time() {
		return travel.t_decel;
	}
};
