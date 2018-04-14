/*
 * helper.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: deeplearning
 */


#include "localisation/helper_functions.h"


MyTimer::MyTimer() {
	maxduration = 0;
	aveduration = 0;
	last_duration = 0;
	count = 0;
}

void MyTimer::start() {
	t1 = high_resolution_clock::now();
}

void MyTimer::stop() {
	t2 = high_resolution_clock::now();
	last_duration = duration_cast < microseconds > (t2 - t1).count();
	update();
}

int MyTimer::getElapsed() {
	high_resolution_clock::time_point t = high_resolution_clock::now();
	int dur = duration_cast < microseconds > (t - t1).count();
	return dur;
}

int MyTimer::restart() {
	this->stop();
	int dur = this->getLastDuration();
	this->start();
	return dur;
}

void MyTimer::update() {

	if (last_duration > 14) {
		count++;

		aveduration = aveduration + (last_duration - aveduration) / count;
	}
	if (last_duration > maxduration) {
		maxduration = last_duration;
	}
}

float angDiff(float a1, float a2) {
	float a = wraparound(a1) - wraparound(a2);
	a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
	return wraparound(a);
}

geometry_msgs::Quaternion getQuaternion(float yaw) {
	tf::Quaternion qt;
	geometry_msgs::Quaternion q;

	tf::Matrix3x3 m;
	m.setEulerZYX(yaw, 0, 0);
	m.getRotation(qt);
	tf::quaternionTFToMsg(qt, q);
	return q;

}

float getYaw(geometry_msgs::Quaternion q) {
	tf::Quaternion qt;
	tf::quaternionMsgToTF(q, qt);
	tf::Matrix3x3 m(qt);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}


float wraparound(float ang) { // [-pi, pi]
	if (equals(ang, 0) || equals(fabs(ang), M_PI))
		return ang;
	if ((ang <= M_PI) && (ang >= -M_PI))
		return ang;
	if (ang > M_PI) {
		ang -= 2 * M_PI;
	}
	if (ang < -M_PI) {
		ang += 2 * M_PI;
	}
	return wraparound(ang);
}
