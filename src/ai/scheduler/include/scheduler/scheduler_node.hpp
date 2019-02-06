
/**	@file scheduler_node.h
* @brief Define the ROS node Scheduler
*		
*		
* @author Alexis CARE
* @author Clément de La Bourdonnaye
*/

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <ros/ros.h>

#include "robot_watcher/Services/RobotServices.h"
#include "robot_watcher/RStatus/Side.h"

#include "action_manager/PerformClient.hpp"

#include "ai_msgs/SetSide.h"
#include "ai_msgs/GetActionToDo.h"
#include "ai_msgs/CurrentActionDone.h"
#include "ai_msgs/SetSchedulerState.h"

#include "scheduler/ActionsParser.hpp"
#include "scheduler/ActionFilePath.hpp"

/**
 * @brief scheduler node class, managing actions and their execution
 */
class Scheduler : public PerformClient {
public:
	Scheduler();

private:
	ros::NodeHandle nh;

	ros::Subscriber side_sub;

	ros::ServiceServer control_srv;

	bool side;
	bool running;

	void setSide(const ai_msgs::SetSide::ConstPtr& msg);
	bool setState(ai_msgs::SetSchedulerState::Request &req, ai_msgs::SetSchedulerState::Response &res);
	
	void stop();
	void resume();
	void nextAction();

	void onFinished(const actionlib::SimpleClientGoalState& state,
		const ai_msgs::PerformResultConstPtr& result) override;
	void onPaused() override;

};
#endif
