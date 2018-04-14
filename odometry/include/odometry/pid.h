#ifndef _PID_H_
#define _PID_H_
#include "odometry/helper.h"

class PIDImpl;
class PID {
public:
// Kp -  proportional gain
// Ki -  Integral gain
// Kd -  derivative gain
// dt -  loop interval time
// max - maximum value of manipulated variable
// min - minimum value of manipulated variable
	PID() {
	}
	void setLinearPID(double max, double min, double integral_max, double Kp,
			double Kd, double Ki);
	void setAngularPID(double max, double min, double integral_max, double Kp,
			double Kd, double Ki);
// Returns the manipulated variable given a setpoint and current process value
	Velocity calculate(Pose_t setpoint, Pose_t pv,bool slowdown =false, bool align=false);
	~PID();
private:
	PIDImpl *ang_pimpl;
	PIDImpl *lin_pimpl;
};
#endif  /*_PID_H_*/
