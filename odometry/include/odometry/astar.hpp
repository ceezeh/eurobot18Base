/*
 Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
 Following tool is licensed under the terms and conditions of the ISC license.
 For more information visit https://opensource.org/licenses/ISC.
 */
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <list>
#include <thread>
#include "odometry/helper.h"

namespace astar {
struct Point {
	int x;
	int y;
	Point(int xt, int yt) {
		x = xt;
		y = yt;
	}
	Point() {
		x = y = 0;
	}
	bool operator==(const astar::Point& p) {
		return(this->x == p.x && this->y == p.y);
	}

};

struct Node {
	uint G, H;
	Point coordinates; // Digital measure as used in A star graph.
	Pose_t trajectory; // Real measurement as setpoint.
	Node *parent;
	bool operator==(const astar::Node& node);
	Node(Point coord_, Node *parent_ = nullptr);
	uint getScore();
};

using uint = unsigned int;
using CoordinateList = std::vector<Point>;
using RealTrajectoryList = std::list<Pose_t>;
using NodeList = std::vector<Node>;
using NodeSet = std::set<Node*>;

Point getDelta(Point source_, Point target_);
uint euclidean(Point source_, Point target_);

class AStar {
public:
	AStar();
	Node* findNodeOnList(NodeSet& nodes_, Point coordinates_);
	void releaseNodes(NodeSet& nodes_);
	void setWorldSize(Point worldSize_);

	void replan(Pose_t, Pose_t);
	Pose_t getSubGoal(Pose_t cur);

	void addCollision(Point coordinates_);
	void removeCollision(Point coordinates_);
	bool detectCollision(astar::Point coordinates_);
	void clearCollisions();

	void setCurrentPose(Pose_t currentPose) {
		this->currentPose = currentPose;
	}
	void setGoalPose(Pose_t goalPose) {
		this->goalPose = goalPose;
	}

private:
	CoordinateList direction, walls;
	RealTrajectoryList trajectory;
	Point worldSize;
	uint directions;
	float resolution;

	CoordinateList findPath(Point source_, Point target_);
	RealTrajectoryList convertPathToRealTrajectory(CoordinateList path, Pose_t,
			Pose_t);

	Point convertToPoint(Pose_t);
	Pose_t convertToRealPose(Point);
	void setBeginReplan(bool replan);
	bool beginReplan;
	bool isInitialised;
	Pose_t currentPose, goalPose;
	std::thread planner;
	void _replan();
};
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
