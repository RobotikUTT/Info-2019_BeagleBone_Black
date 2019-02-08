#ifndef ACTION_PERFORMER_H
#define ACTION_PERFORMER_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include "ai_msgs/PerformAction.h"
#include "ai_msgs/Argument.h"
#include "ai_msgs/ComputeActionPoint.h"
#include "ai_msgs/RobotStatus.h"

#include "action_manager/ActionPoint.h"
#include "action_manager/Action.hpp"

#include "can_msgs/Finish.h"
#include "can_msgs/Point.h"
#include "can_msgs/Status.h"

#include "procedures_msgs/OrPoint.h"

#include "robot_watcher/Services/RobotServices.h"
#include "robot_watcher/RStatus/State.h"

typedef actionlib::SimpleActionServer<ai_msgs::PerformAction> PerformActionSrv;

/**
 * Represent an performer for a specific action, it advertise a service for
 * predicting position and an action server to run the action.
 * 
 * It provide simple interface for creating robot action, and avoid tedious ROS objects
 * manipulation.
 */
class ActionPerformer
{
public:
	ActionPerformer(std::string name);
	
protected:
	// ROS nodehandle is protected to let child use ros
	ros::NodeHandle nh;

	// Function defined by inherited actions
	virtual ActionPoint computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, Point robotPos) = 0;
	virtual void start() = 0;
	virtual void cancel() {};

	double getArg(std::string name, double defaultValue = 0, std::vector<ai_msgs::Argument>* args = NULL);
	bool hasArg(std::string name, std::vector<ai_msgs::Argument>* args = NULL);

	// Function managing the action
	void actionPerformed();
	void actionPaused();
	void ready();
private:
	std::vector<ai_msgs::Argument> _args;

	// Name of the perfomer
	std::string name;

	// Ros objects
	PerformActionSrv* actionServer;
	ros::ServiceServer actionPointSrv;
	ros::Subscriber robotWatcherSub;

	bool _computeActionPoint(
		ai_msgs::ComputeActionPoint::Request& req,
		ai_msgs::ComputeActionPoint::Response& res
	);

	void onGoal();
	void onPreempt();

	void onRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
};

#endif
