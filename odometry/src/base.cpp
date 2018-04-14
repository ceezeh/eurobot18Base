/*
 * This program receives angular and velocity commands and relays to the robot's base.
 * It also periodically publishes its current estimate of the robot's velocity.
 */

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <chrono>
#include "odometry/helper.h"
#include "odometry/wiringSerial.h"
#include "odometry/robot.h"
#include "odometry/pid.h"
#include "odometry/astar.hpp"

using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "base");
	ros::NodeHandle n;

	Robot robot(n);
	PID pid; // PID Control Helper Class

	astar::AStar astar; // Planning Helper Class.

	// PID for linear velocity
	double max = 127;
	double min = -127;
	double Kp_v = 250;
	double Kd_v = 0;
	double Ki_v = 0;
	double integral_max = 50;
	double Kp_w = 60;
	double Kd_w = 0;
	double Ki_w = 0;
	pid.setLinearPID(max, min, integral_max, Kp_v, Kd_v, Ki_v);
	pid.setAngularPID(max, min, integral_max, Kp_w, Kd_w, Ki_w);

	ros::Rate loop_rate(50);

	MyTimer odomTimer;
	MyTimer motionTimer;
	odomTimer.start();
	motionTimer.start();
	float motionPeriod = .05; // seconds
	float odomPeriod = .02; // seconds

	while (ros::ok()) {

		if (motionTimer.getElapsed() / 1e6 > motionPeriod) {
			motionTimer.restart();

			if (robot.newOdom() && !robot.hasIndicatedReached()) {
				cout << "New Odom" << endl;
				//				cout << "New Goal: " << robot.newGoal()<<endl;
				if (robot.newGoal() || robot.nearObstacle()) {
					// replan.
					cout << "Begin Replanning" << endl;
					astar.replan(robot.currentLocation(), robot.goalLocation()); //TODO:Should run on a separate thread
				}

				Pose_t subGoal = astar.getSubGoal(robot.currentLocation());
				cout << "Sub Goal[x:" << subGoal.x << ", y:" << subGoal.y
						<< ", th: " << subGoal.th << "]" << endl;
				cout << " Robot location[x:" << robot.currentLocation().x
						<< ", y:" << robot.currentLocation().y << ", th:"
						<< robot.currentLocation().th << "]" << endl;

				Velocity vel;
				if (subGoal == robot.goalLocation()) {
					// Turn!
					bool align = false;
					bool slowdown = true;

					if (robot.currentLocation().closePointTo(
							robot.goalLocation())) {
						align = true;
					}
					if (robot.isReached()) {
						robot.move(Velocity(0, 0));
						robot.indicateReached();
					} else {
						vel = pid.calculate(robot.goalLocation(),
								robot.currentLocation(), slowdown, align);
					}
				} else {
					vel = pid.calculate(subGoal, robot.currentLocation(), false,
							false);
				}
				cout << "[PID] v=" << vel.v << ", w=" << vel.w << endl;
				robot.move(vel);

			} else {
				cout << "\rStopping";
				robot.move(Velocity(0, 0));
				if (robot.isReached()) {
					robot.indicateReached();
					cout << "\rRobot has reached destination..";
					cout << " Robot location[x:" << robot.currentLocation().x
							<< ", y:" << robot.currentLocation().y << "]"
							<< endl;
					cout << " Goal location[x:" << robot.goalLocation().x
							<< ", y:" << robot.goalLocation().y << "]" << endl;
				}
			}
		}

		if (odomTimer.getElapsed() / 1e6 > odomPeriod) {
			odomTimer.restart();
			Velocity velIntegral = robot.getVelocityIntegration();
			robot.publishSpeed(velIntegral);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

// Stop motors before exit
	return 0;
}

