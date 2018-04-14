#ifndef _PID_SOURCE_
#define _PID_SOURCE_
#include <iostream>
#include <cmath>
#include "odometry/pid.h"
using namespace std;
class PIDImpl {
public:
	PIDImpl(double max, double min, double integral_max, double Kp, double Kd,
			double Ki);
	~PIDImpl();
	double calculate(double error);
private:
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
	double _integral_max;
	MyTimer timer;
};

Velocity PID::calculate(Pose_t setpoint, Pose_t pv, bool slowdown, bool align) {
	float lin_dist = sqrt(
			pow(setpoint.x - pv.x, 2) + pow(setpoint.y - pv.y, 2));

	/*
	 * If robot is facing goal, then velocity should be +ve,
	 * else -ve.
	 */

	float heading = pv.th;
	float bearingToGoal = atan2(setpoint.y - pv.y, setpoint.x - pv.x);
	float angdiff = angDiff(bearingToGoal, heading);

	bool front = abs(angDiff(heading, bearingToGoal) < M_PI / 2);
	if (!front) {
		lin_dist *= -1;
	}
	float ang_dist = angDiff(setpoint.th, pv.th);

	float w, v;
	if (abs(angdiff) < .2) { // Warning: Using hard coded value.
		if (!slowdown) {
			v = copysign(100, lin_dist);
		} else {
			v = lin_pimpl->calculate(lin_dist);
			cout<<"Slowing down..."<<endl;
		}
	} else {
		v = 0;
	}
	if (align) {
		w = ang_pimpl->calculate(ang_dist);
		cout << "Lin_dist: " << lin_dist << ", [Align] angdist: " << ang_dist
				<< endl;
	} else {
		w = ang_pimpl->calculate(angdiff); // In this case, probably just use proportional term.
		cout << "Lin_dist: " << lin_dist << ", angdiff: " << angdiff << endl;
	}

	if (v > 126)
		v = 126;
	if (v < -126)
		v = -126;
	if (w > 126)
		w = 126;
	if (w < -126)
		w = -126;
	return {v,w};
}

void PID::setLinearPID(double max, double min, double integral_max, double Kp,
		double Kd, double Ki) {
	lin_pimpl = new PIDImpl(max, min, integral_max, Kp, Kd, Ki);
}

void PID::setAngularPID(double max, double min, double integral_max, double Kp,
		double Kd, double Ki) {
	ang_pimpl = new PIDImpl(max, min, integral_max, Kp, Kd, Ki);
}

PID::~PID() {
	delete lin_pimpl;
	delete ang_pimpl;
}
/**
 * Implementation
 */
PIDImpl::PIDImpl(double max, double min, double integral_max, double Kp,
		double Kd, double Ki) :

		_max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(
				0), _integral_max(integral_max) {
	timer.start();
}

double PIDImpl::calculate(double error) {
	double _dt = timer.restart() / 1e6;
// Proportional term
	double Pout = _Kp * error;
// Integral term
	_integral = 0.8 * _integral + 0.2 * error * _dt;
	if (abs(_integral) > _integral_max) {
		_integral = copysign(_integral_max, _integral);
	}
	double Iout = _Ki * _integral;
// Derivative term

	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;
// Calculate total output
	double output = Pout + Iout + Dout;
// Restrict to max/min
	if (output > _max)
		output = _max;
	else if (output < _min)
		output = _min;
// Save error to previous error
	_pre_error = error;
	return output;
}
PIDImpl::~PIDImpl() {
}
#endif
