/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller.
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "controller_indi.h"
#include "math3d.h"

static float thrust_threshold = 300.0f;
static float bound_control_input = 32000.0f;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;
struct FloatRates body_rates;
static vector_t refOuterINDI;				// Reference values from outer loop INDI
static bool outerLoopActive = true ; 		// if 1, outer loop INDI is activated

static struct IndiVariables indi = {
		.g1 = {STABILIZATION_INDI_G1_P, STABILIZATION_INDI_G1_Q, STABILIZATION_INDI_G1_R},
		.g2 = STABILIZATION_INDI_G2_R,
		.reference_acceleration = {
				STABILIZATION_INDI_REF_ERR_P,
				STABILIZATION_INDI_REF_ERR_Q,
				STABILIZATION_INDI_REF_ERR_R,
				STABILIZATION_INDI_REF_RATE_P,
				STABILIZATION_INDI_REF_RATE_Q,
				STABILIZATION_INDI_REF_RATE_R
		},
		.act_dyn = {STABILIZATION_INDI_ACT_DYN_P, STABILIZATION_INDI_ACT_DYN_Q, STABILIZATION_INDI_ACT_DYN_R},
		.filt_cutoff = STABILIZATION_INDI_FILT_CUTOFF,
		.filt_cutoff_r = STABILIZATION_INDI_FILT_CUTOFF_R,
};

static inline void float_rates_zero(struct FloatRates *fr) {
	fr->p = 0.0f;
	fr->q = 0.0f;
	fr->r = 0.0f;
}

void indi_init_filters(void)
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff);
	float tau_r = 1.0f / (2.0f * M_PI_F * indi.filt_cutoff_r);
	float tau_axis[3] = {tau, tau, tau_r};
	float sample_time = 1.0f / ATTITUDE_RATE;
	// Filtering of gyroscope and actuators
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&indi.u[i], tau_axis[i], sample_time, 0.0f);
		init_butterworth_2_low_pass(&indi.rate[i], tau_axis[i], sample_time, 0.0f);
	}
}

/**
 * @brief Update butterworth filter for p, q and r of a FloatRates struct
 *
 * @param filter The filter array to use
 * @param new_values The new values
 */
static inline void filter_pqr(Butterworth2LowPass *filter, struct FloatRates *new_values)
{
	update_butterworth_2_low_pass(&filter[0], new_values->p);
	update_butterworth_2_low_pass(&filter[1], new_values->q);
	update_butterworth_2_low_pass(&filter[2], new_values->r);
}

/**
 * @brief Caclulate finite difference form a filter array
 * The filter already contains the previous values
 *
 * @param output The output array
 * @param filter The filter array input
 */
static inline void finite_difference_from_filter(float *output, Butterworth2LowPass *filter)
{
	for (int8_t i = 0; i < 3; i++) {
		output[i] = (filter[i].o[0] - filter[i].o[1]) * ATTITUDE_RATE;
	}
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}


void controllerINDIInit(void)
{
	/*
	 * TODO
	 * Can this also be called during flight, for instance when switching controllers?
	 * Then the filters should not be reset to zero but to the current values of sensors and actuators.
	 */
	float_rates_zero(&indi.angular_accel_ref);
	float_rates_zero(&indi.u_act_dyn);
	float_rates_zero(&indi.u_in);

	// Re-initialize filters
	indi_init_filters();

	attitudeControllerInit(ATTITUDE_UPDATE_DT);
	positionControllerInit();
	positionControllerINDIInit();
}

bool controllerINDITest(void)
{
	bool pass = true;

	pass &= attitudeControllerTest();

	return pass;
}

void controllerINDI(control_t *control, const setpoint_t *setpoint,
	const sensorData_t *sensors,
	const state_t *state,
	const uint32_t tick)
{
	control->controlMode = controlModeLegacy;

	//The z_distance decoder adds a negative sign to the yaw command, the position decoder doesn't
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		// Rate-controled YAW is moving YAW angle setpoint
		if (setpoint->mode.yaw == modeVelocity) {
			attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT; //if line 140 (or the other setpoints) in crtp_commander_generic.c has the - sign remove add a -sign here to convert the crazyfly coords (ENU) to INDI  body coords (NED)
			while (attitudeDesired.yaw > 180.0f)
				attitudeDesired.yaw -= 360.0f;
			while (attitudeDesired.yaw < -180.0f)
				attitudeDesired.yaw += 360.0f;

			attitudeDesired.yaw = radians(attitudeDesired.yaw); //convert to radians
		} else {
			attitudeDesired.yaw = setpoint->attitude.yaw;
			attitudeDesired.yaw = capAngle(attitudeDesired.yaw); //use the capangle as this is also done in velocity mode
			attitudeDesired.yaw = -radians(attitudeDesired.yaw); //convert to radians and add negative sign to convert from ENU to NED
		}
	}

	if (RATE_DO_EXECUTE(POSITION_RATE, tick) && !outerLoopActive) {
		positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
	}

	/*
	 * Skipping calls faster than ATTITUDE_RATE
	 */
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

		// Call outer loop INDI (position controller)
		if (outerLoopActive) {
			positionControllerINDI(sensors, setpoint, state, &refOuterINDI);
		}

		// Switch between manual and automatic position control
		if (setpoint->mode.z == modeDisable) {
				// INDI position controller not active, INDI attitude controller is main loop
				actuatorThrust = setpoint->thrust;
		} else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				actuatorThrust = refOuterINDI.z;
			}
		}
		if (setpoint->mode.x == modeDisable) {

				// INDI position controller not active, INDI attitude controller is main loop
				attitudeDesired.roll = radians(setpoint->attitude.roll); //no sign conversion as CF coords is equal to NED for roll

		}else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				attitudeDesired.roll = refOuterINDI.x; //outer loop provides radians
			}
		}

		if (setpoint->mode.y == modeDisable) {

				// INDI position controller not active, INDI attitude controller is main loop
				attitudeDesired.pitch = radians(setpoint->attitude.pitch); //no sign conversion as CF coords use left hand for positive pitch.

		}else{
			if (outerLoopActive) {
				// INDI position controller active, INDI attitude controller becomes inner loop
				attitudeDesired.pitch = refOuterINDI.y; //outer loop provides radians
			}
		}

		//Proportional controller on attitude angles [rad]
		rateDesired.roll 	= indi.reference_acceleration.err_p*(attitudeDesired.roll - radians(state->attitude.roll));
		rateDesired.pitch 	= indi.reference_acceleration.err_q*(attitudeDesired.pitch - radians(state->attitude.pitch));
		rateDesired.yaw 	= indi.reference_acceleration.err_r*(attitudeDesired.yaw - (-radians(state->attitude.yaw))); //negative yaw ENU  ->  NED

		// For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
		// value. Also reset the PID to avoid error buildup, which can lead to unstable
		// behavior if level mode is engaged later
		if (setpoint->mode.roll == modeVelocity) {
			rateDesired.roll = radians(setpoint->attitudeRate.roll);
			attitudeControllerResetRollAttitudePID();
		}
		if (setpoint->mode.pitch == modeVelocity) {
			rateDesired.pitch = radians(setpoint->attitudeRate.pitch);
			attitudeControllerResetPitchAttitudePID();
		}

		/*
		 * 1 - Update the gyro filter with the new measurements.
		 */

		body_rates.p = radians(sensors->gyro.x);
		body_rates.q = -radians(sensors->gyro.y); //Account for gyro measuring pitch rate in opposite direction relative to both the CF coords and INDI coords
		body_rates.r = -radians(sensors->gyro.z); //Account for conversion of ENU -> NED

		filter_pqr(indi.rate, &body_rates);

		/*
		 * 2 - Calculate the derivative with finite difference.
		 */

		finite_difference_from_filter(indi.rate_d, indi.rate);

		/*
		 * 3 - same filter on the actuators (or control_t values), using the commands from the previous timestep.
		 */
		filter_pqr(indi.u, &indi.u_act_dyn);


		/*
		 * 4 - Calculate the desired angular acceleration by:
		 * 4.1 - Rate_reference = P * attitude_error, where attitude error can be calculated with your favorite
		 * algorithm. You may even use a function that is already there, such as attitudeControllerCorrectAttitudePID(),
		 * though this will be inaccurate for large attitude errors, but it will be ok for now.
		 * 4.2 Angular_acceleration_reference = D * (rate_reference – rate_measurement)
		 */

		//Calculate the attitude rate error, using the unfiltered gyroscope measurements (only the preapplied filters in bmi088)
		float attitude_error_p = rateDesired.roll - body_rates.p;
		float attitude_error_q = rateDesired.pitch - body_rates.q;
		float attitude_error_r = rateDesired.yaw - body_rates.r;

		//Apply derivative gain
		indi.angular_accel_ref.p = indi.reference_acceleration.rate_p * attitude_error_p;
		indi.angular_accel_ref.q = indi.reference_acceleration.rate_q * attitude_error_q;
		indi.angular_accel_ref.r = indi.reference_acceleration.rate_r * attitude_error_r;

		/*
		 * 5. Update the For each axis: delta_command = 1/control_effectiveness * (angular_acceleration_reference – angular_acceleration)
		 */

		//Increment in angular acceleration requires increment in control input
		//G1 is the control effectiveness. In the yaw axis, we need something additional: G2.
		//It takes care of the angular acceleration caused by the change in rotation rate of the propellers
		//(they have significant inertia, see the paper mentioned in the header for more explanation)
		indi.du.p = 1.0f / indi.g1.p * (indi.angular_accel_ref.p - indi.rate_d[0]);
		indi.du.q = 1.0f / indi.g1.q * (indi.angular_accel_ref.q - indi.rate_d[1]);
		indi.du.r = 1.0f / (indi.g1.r + indi.g2) * (indi.angular_accel_ref.r - indi.rate_d[2] + indi.g2 * indi.du.r);


		/*
		 * 6. Add delta_commands to commands and bound to allowable values
		 */

		indi.u_in.p = indi.u[0].o[0] + indi.du.p;
		indi.u_in.q = indi.u[1].o[0] + indi.du.q;
		indi.u_in.r = indi.u[2].o[0] + indi.du.r;

		//bound the total control input
		indi.u_in.p = clamp(indi.u_in.p, -1.0f*bound_control_input, bound_control_input);
		indi.u_in.q = clamp(indi.u_in.q, -1.0f*bound_control_input, bound_control_input);
		indi.u_in.r = clamp(indi.u_in.r, -1.0f*bound_control_input, bound_control_input);

		//Propagate input filters
		//first order actuator dynamics
		indi.u_act_dyn.p = indi.u_act_dyn.p + indi.act_dyn.p * (indi.u_in.p - indi.u_act_dyn.p);
		indi.u_act_dyn.q = indi.u_act_dyn.q + indi.act_dyn.q * (indi.u_in.q - indi.u_act_dyn.q);
		indi.u_act_dyn.r = indi.u_act_dyn.r + indi.act_dyn.r * (indi.u_in.r - indi.u_act_dyn.r);

	}

	indi.thrust = actuatorThrust;

	//Don't increment if thrust is off
	//TODO: this should be something more elegant, but without this the inputs
	//will increment to the maximum before even getting in the air.
	if(indi.thrust < thrust_threshold) {
		float_rates_zero(&indi.angular_accel_ref);
		float_rates_zero(&indi.u_act_dyn);
		float_rates_zero(&indi.u_in);

		if(indi.thrust == 0){
			attitudeControllerResetAllPID();
			positionControllerResetAllPID();

			// Reset the calculated YAW angle for rate control
			attitudeDesired.yaw = -state->attitude.yaw;
		}
	}

	/*  INDI feedback */
	control->thrust = indi.thrust;
	control->roll = indi.u_in.p;
	control->pitch = indi.u_in.q;
	control->yaw  = indi.u_in.r;

}
