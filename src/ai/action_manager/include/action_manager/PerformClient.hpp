#ifndef PERFORM_CLIENT_HPP
#define PERFORM_CLIENT_HPP

#include <iostream>
#include <ros/ros.h>

#include "procedures_msgs/OrPoint.h"

#include <actionlib/client/simple_action_client.h>

#include "ai_msgs/PerformAction.h"

#include "action_manager/Point.h"
#include "action_manager/ActionPoint.h"
#include "action_manager/AtomicAction.hpp"
#include "action_manager/Action.hpp"

typedef actionlib::SimpleActionClient<ai_msgs::PerformAction> PerformActionClt;

class PerformClient {
public:
	PerformClient();

	virtual void onFinished(const actionlib::SimpleClientGoalState& state,
		const ai_msgs::PerformResultConstPtr& result) = 0;
	virtual void onPaused() = 0;
protected:
	void performAction(AtomicAction& action, procedures_msgs::OrPoint& robot_pos);

	ros::NodeHandle nh;
private:
	PerformActionClt* client;
};

#endif