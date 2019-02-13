
/**
 * @file scheduler_node.h
 * @brief Define the ROS node Scheduler
 *
 * @author Clément de La Bourdonnaye
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <ros/ros.h>

#include "action_manager/PerformClient.hpp"
#include "action_manager/Point.hpp"

#include "ai_msgs/Side.h"
#include "ai_msgs/SetSchedulerState.h"

#include "can_msgs/Point.h"

#include "scheduler/ActionsParser.hpp"
#include "scheduler/ActionTools.hpp"
#include "scheduler/ActionFilePath.hpp"

using ai_msgs::Side;
using ai_msgs::SetSchedulerState;
using ai_msgs::NodeStatus;

/**
 * @brief scheduler node class, managing actions and their execution
 */
class Scheduler : public PerformClient {
public:
	Scheduler();

private:
	ros::NodeHandle nh;
	ros::ServiceServer control_srv;
	ros::Subscriber robotPosition_sub;
	
	ActionPtr rootAction;
	ActionPtr currentAction;

	Point robotPosition;

	bool side;
	bool running;

	bool setState(SetSchedulerState::Request &req, SetSchedulerState::Response &res);
	void setRobotPosition(const can_msgs::Point::ConstPtr& msg);

	void nextAction();

	void onFinished(const actionlib::SimpleClientGoalState& state,
		const ai_msgs::PerformResultConstPtr& result) override;
	void onPaused() override;

};
#endif
