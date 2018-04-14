#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <unistd.h>
#include <sstream>
#include <cmath>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include "odometry/helper.h"
#include "odometry/md49.h"

using namespace md49;

class Robot {

#define DIST_THRES 0.4
#define SENSOR_0_BIT 0
#define SENSOR_1_BIT 1
#define SENSOR_2_BIT 2
#define ODOM_BIT 3
#define GOAL_BIT 4
public:
	Robot(ros::NodeHandle& n);
	~Robot();
	Velocity getVelocityIntegration();
	void turnUntil(int fd, float rad);
	void publishSpeed(Velocity v);
	void initialiseGoal();
	bool newGoal();
	bool newOdom();
	bool nearObstacle();
	void move(Velocity v);

	Pose_t currentLocation() {
		return {odom.pose.pose.position.x, odom.pose.pose.position.y, getYaw(odom.pose.pose.orientation)};
	}

	Pose_t goalLocation() {

		return Pose_t(goal.position.x, goal.position.y,
				getYaw(goal.orientation));
	}

	// Callbacks
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t);
	void irCallback(const sensor_msgs::LaserScan::ConstPtr& ir);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_t);

	bool isReached() {
		return (this->goalLocation()).closeTo(this->currentLocation());

	}
	bool hasIndicatedReached() {
		return this->indicateFlag;
	}
	void indicateReached();
private:
	md49::MD49* baseController;
	ros::NodeHandle n;
	ros::Publisher speed_pub;
	ros::Publisher status_pub;
	ros::Subscriber odom_sub;
	ros::Subscriber sensor_sub;
	ros::Subscriber goal_sub;
	ros::CallbackQueue callbackQueue;
	ros::AsyncSpinner* spinner;
	uint8_t _dataflag;
	uint8_t _sensorflag;
	bool indicateFlag;

	void clearSensorFlag() {
		_sensorflag = 0;
	}
	void setSensorFlag(int bit) {
		_sensorflag |= (1 << bit);
	}
	bool getSensorFlag(int bit) {
		return _sensorflag & (1 << bit);
	}
	bool getSensorFlag() {
		return _sensorflag;
	}

	void clearDataFlag() {
		_dataflag = 0;
	}
	void clearDataBit(int bit) {
		_dataflag &= ~(1 << bit);
	}

	void setDataFlag(int bit) {
		_dataflag |= (1 << bit);
	}
	bool getDataFlag(int bit) {
		return _dataflag & (1 << bit);
	}
	geometry_msgs::Pose goal;
	nav_msgs::Odometry odom;

	float _getAngDiff();
};
#endif  /*_ROBOT_H_*/
