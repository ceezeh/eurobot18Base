#include "odometry/robot.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>

Robot::Robot(ros::NodeHandle& nt) :
		n(nt) {
	baseController = new MD49();
	clearDataFlag();
	clearSensorFlag();
	this->indicateFlag = true;
	speed_pub = n.advertise < geometry_msgs::Twist > ("/base_speed", 10);
	status_pub = n.advertise < std_msgs::Int8 > ("/task_status", 10);

	// Goal
	ros::SubscribeOptions ops = ros::SubscribeOptions::create
			< geometry_msgs::PoseStamped > ("/my_robot/goal", // topic name
	10, // queue length
	boost::bind(&Robot::goalCallback, this, _1), // callback
	ros::VoidPtr(), // tracked object, we don't need one thus NULL
	&callbackQueue // pointer to callback queue object
			);
	goal_sub = n.subscribe(ops);

	// IRs
	ops = ros::SubscribeOptions::create < sensor_msgs::LaserScan > ("ir", // topic name
	10, // queue length
	boost::bind(&Robot::irCallback, this, _1), // callback
	ros::VoidPtr(), // tracked object, we don't need one thus NULL
	&callbackQueue // pointer to callback queue object
			);

	sensor_sub = n.subscribe(ops);

	// Odom
	ops = ros::SubscribeOptions::create < nav_msgs::Odometry
			> ("/my_robot/odom", // topic name
			1, // queue length
			boost::bind(&Robot::odomCallback, this, _1), // callback
			ros::VoidPtr(), // tracked object, we don't need one thus NULL
			&callbackQueue // pointer to callback queue object
			);

	odom_sub = n.subscribe(ops);

	// Spinner
	spinner = new ros::AsyncSpinner(0, &callbackQueue);
	spinner->start();
	baseController->displayDecoderValues();
	this->initialiseGoal();
}

Robot::~Robot() {
	delete spinner;
	delete baseController;
}

void Robot::publishSpeed(Velocity vel) {
	geometry_msgs::Twist cmd;
	cmd.angular.z = vel.w;
	cmd.linear.x = vel.v;
	speed_pub.publish(cmd);
}

void Robot::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_t) {
	if (goal_t->header.frame_id == GOTO_ID) { // Only allow goto commands to pass.
		goal = geometry_msgs::Pose(goal_t->pose);
		cout << "\rReceived new destination..";
		setDataFlag (GOAL_BIT);
		cout << "Exiting goal callback...";
		this->indicateFlag = false;
	}
}

void Robot::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_t) {
	odom = nav_msgs::Odometry(*odom_t);
	setDataFlag (ODOM_BIT);
}

/*
 * Checks flag and resets it.
 */
bool Robot::newGoal() {
	bool val = getDataFlag(GOAL_BIT);
	clearDataBit (GOAL_BIT);
	return val;
}

bool Robot::newOdom() {
	bool val = getDataFlag(ODOM_BIT);
	clearDataBit (ODOM_BIT);
	return val;
}

bool Robot::nearObstacle() {
	return (getSensorFlag() != 0);
}

float Robot::_getAngDiff() {
	float heading = getYaw(odom.pose.pose.orientation);
	float bearingToGoal = atan2(goal.position.y - odom.pose.pose.position.y,
			goal.position.x - odom.pose.pose.position.x);
	float angdiff = angDiff(heading, bearingToGoal);
	return angdiff;
}

void Robot::initialiseGoal() {
	while (!getDataFlag(ODOM_BIT)) {
		cout << "\rWaiting for new odometry data to initialise goal...";
		usleep(1000);
	}
	goal = geometry_msgs::Pose(odom.pose.pose);
}

void Robot::irCallback(const sensor_msgs::LaserScan::ConstPtr& ir) {

	float dist = ir->ranges[0];

	if (strcmp(ir->header.frame_id.c_str(), "ir0") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				setSensorFlag (SENSOR_0_BIT);
			}
		setDataFlag (SENSOR_0_BIT);
	} else if (strcmp(ir->header.frame_id.c_str(), "ir4") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				setSensorFlag (SENSOR_1_BIT);
			}
		setDataFlag (SENSOR_1_BIT);
	} else if (strcmp(ir->header.frame_id.c_str(), "ir1") == 0) {
		if (!isinf(dist))
			if (dist < DIST_THRES) {
				setSensorFlag (SENSOR_2_BIT);
			}
		setDataFlag (SENSOR_2_BIT);
	}
}

Velocity Robot::getVelocityIntegration() {
	Velocity vel;
	baseController->getVelocityIntegration(vel.v, vel.w);
	return vel;
}

void Robot::move(Velocity vel) {
	int8_t v = int8_t(vel.v) & 255;
	int8_t w = int8_t(vel.w) & 255;
	baseController->driveMotors(v, w);
}

void Robot::indicateReached() {
	if (this->isReached()  && !this->indicateFlag) {
		std_msgs::Int8 status;
		status.data = 1;
		status_pub.publish(status);
		this->indicateFlag = true;
	}
}

