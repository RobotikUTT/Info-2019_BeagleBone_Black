#ifndef PERFORM_CLIENT_HPP
#define PERFORM_CLIENT_HPP

#include <iostream>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include "ai_msgs/PerformAction.h"

#include "node_watcher/Node.hpp"

#include "action_manager/AtomicAction.hpp"
#include "action_manager/ActionBlock.hpp"

#include "geometry_msgs/Pose2D.h"
#include "ai_msgs/NodeRequirement.h"

typedef actionlib::SimpleActionClient<ai_msgs::PerformAction> PerformActionClt;

using actionlib::SimpleClientGoalState;

class PerformClient : public Node {
public:
	PerformClient(std::string name, std::string package);

	virtual void onFinished(const actionlib::SimpleClientGoalState& state,
		const ai_msgs::PerformResultConstPtr& result) = 0;
	virtual void onPaused() = 0;

protected:
	void performAction(AtomicActionPtr action, Pose2D robot_pos);
	void cancelAction();

	void saveRequired(ActionPtr action);
private:
	PerformActionClt* client;
};

#endif