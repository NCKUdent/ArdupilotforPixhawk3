// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ROLL_CONTROLLER_H__
#define __AP_ROLL_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <math.h>

class AP_RollController {
public:
	AP_RollController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms) :
		aparm(parms),
        _ahrs(ahrs)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);
	int32_t custom_get_servo_out(int32_t angle_err, bool disable_integrator);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	const AP_Vehicle::FixedWing &aparm;
	AP_Float _tau;
	AP_Float _K_P;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Int16 _max_rate;
    AP_Int16  _imax;
	uint32_t _last_t;
	float _last_out;
	float _last_out_deg;

	float _integrator;
	float roll_I_integrator;
	float roll_D_derivative;
	float rate_error_prior;
//	float Filter_Dstate;
//	float FilterCoefficient;

	int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);
	int32_t _custom_get_rate_out(float desired_rate, bool disable_integrator);

	AP_AHRS &_ahrs;

};

#endif // __AP_ROLL_CONTROLLER_H__
