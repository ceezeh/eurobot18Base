#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "localisation/helper_functions.h"
class Kalman {
public:
	Kalman();
	void init(double x, double y);
	void prediction(double ds, double dth);
	void update(Pose_t observation, double std_pred[], double std_obs[]);
	geometry_msgs::Pose getPosition();
	bool isInitialised(){
		return initialised;
	}
private:
	Pose_t p;
	Pose_t last;
	bool isCombined;
	bool initialised;
};


#endif
