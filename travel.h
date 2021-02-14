#define MAX_V 80000
#define MAX_A 1000000

struct Travel {
	float p_start = 0;
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

struct TravelItem {
	float p = 0; // position
	float v = 0; // velocity
};

void travel_create(Travel* result, TravelItem *travel_item, float maxv, float maxa, float t_start, float p) {
	float v0 = travel_item->v;
	float d_dest = p - travel_item->p;

	static float 
		dd_0,
		dd_accel,
		dd_cruise,
		dtotal,
		dt_0,
		dt_reverse,
		dt_accel,
		dt_cruise,
		dt_decel,
		ttop,
		dtop
	;

	if (v0 * d_dest > 0) {
		if (abs(d_dest) < abs(0.5 * maxa * pow((v0 / maxa), 2))) {
			// TODO: impossible instruction. Unable to decel in time. 
			return;
		}
	}

	static int dir;

	result->d_dest = d_dest;
	result->p_start = travel_item->p;
	result->t_start = t_start;
	result->v0 = v0;
	result->maxa = maxa;

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

	result->t_accel = dt_reverse + dt_accel;
	result->t_cruise = result->t_accel + dt_cruise;
	result->t_decel = result->t_cruise + dt_decel;
	result->d_accel = dd_accel;
	result->d_cruise = result->d_accel + dd_cruise;
	result->maxv = maxv;
	result->maxa = maxa;
	result->dir = dir;
}

void travel_tick(Travel *travel, TravelItem *travel_item,  float t) {
	static float p;
	static float dt;

	t = t - travel->t_start;
	if (t > travel->t_decel) {
		p = travel->d_dest;
		travel_item->v = 0;
	} else if (t < travel->t_accel) {
		// accelerating
		p = travel->v0 * t + 0.5f * travel->maxa * travel->dir * pow(t, 2);
		travel_item->v = travel->v0 + travel->maxa * travel->dir * t;
	} else if (t < travel->t_cruise) {
		// cruising
		dt = t - travel->t_accel;
		p = travel->d_accel * travel->dir + travel->maxv * travel->dir * (dt);
		travel_item->v = travel->maxv * travel->dir;
	} else {
		// decelerating
		dt = t - travel->t_cruise;
		p = travel->d_cruise * travel->dir + travel->maxv * travel->dir * (dt) + 0.5f * (travel->maxa * (-travel->dir)) * pow(dt, 2);
		travel_item->v = travel->maxv * travel->dir - travel->maxa * travel->dir * dt;
	}

	travel_item->p = travel->p_start + p;
}

