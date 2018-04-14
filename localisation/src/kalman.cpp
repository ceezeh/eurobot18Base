#include "localisation/kalman.h"

void Kalman::prediction(double ds, double dth) {
	if (abs(ds) > 0.001 || abs(dth) > 0.001) {
		isCombined = true;
	}
	double new_x = p.x + ds * cos(p.th + dth);
	double new_y = p.y + ds * sin(p.th + dth);
	double new_theta = p.th + dth;

	p.x = new_x;
	p.y = new_y;
	p.th = new_theta;
}

void Kalman::update(Pose_t observation, double std_pred[], double std_obs[]) {

	//
	if (!isCombined) {
		std_pred[1] = 0;
	}

	double x0 = last.x;
	double y0 = last.y;

	p.x = (std_obs[0] * p.x + std_pred[0] * observation.x)
			/ (std_pred[0] + std_obs[0]);
	p.y = (std_obs[0] * p.y + std_pred[0] * observation.y)
			/ (std_pred[0] + std_obs[0]);

	double thObs = atan2(p.y - y0, p.x - x0);
	p.th = angDiff(std_obs[1] * p.th / (std_obs[1] + std_pred[1]),
			-std_pred[1] * thObs / (std_obs[1] + std_pred[1]));

	if (isCombined) {
		isCombined = false;
		last = p;
	}
}

Kalman::Kalman() {
	p = {0, 0, 0};
	isCombined = false;
	initialised = false;
}
void Kalman::init(double x, double y) {
	last = {x, y, 0};
	p = last;
	initialised = true;
}
geometry_msgs::Pose Kalman::getPosition() {
	geometry_msgs::Pose pt;
	if (isinf(p.x) || isnan(p.x) || isinf(p.y) || isnan(p.y) || isnan(p.th)
			|| isnan(p.th)) {
		pt.orientation.w = 1000000; // Hack to represent null value.
	} else {

		pt.position.x = p.x;
		pt.position.y = p.y;
		cout << "\rx: " << p.x << ", y: " << p.y << ", th: " << p.th;
		pt.orientation = getQuaternion(p.th);
	}
	return pt;
}
