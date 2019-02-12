/** @file controller_node.h
*	@brief controller_node h file
*		
*/
#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>

#include "node_watcher/Node.hpp"

#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/Side.h"

#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/ProximityStop.h"
#include "ai_msgs/StartRobot.h"
#include "ai_msgs/PointsScored.h"
#include "ai_msgs/SetSchedulerState.h"

#include "can_msgs/Point.h"
#include "can_msgs/STMStatus.h"
#include "can_msgs/SonarDistance.h"
#include "can_msgs/Frame.h"
#include "can_msgs/CurrSpeed.h"
#include "can_msgs/RobotBlocked.h"

#include "procedures_msgs/MoveAction.h"

#include "std_msgs/Int8.h"

/**
 * @defgroup Controller The Controller package
 * @{
 */

#define SONAR_MIN_DIST_FORWARD 30 // in cm
#define SONAR_MIN_DIST_BACKWARD	10 // in cm

using ros::Subscriber;
using ros::Publisher;
using ros::ServiceClient;

// Use ai_msgs namespace to simplify usage
using namespace ai_msgs;

using can_msgs::Point;

/**
 * @brief process inputs and change robot behavior according to that
 */
class Controller : public Node
{
public:
	Controller();

private:
	Subscriber status_sub;
	Subscriber nodes_status_sub;
	Subscriber sonar_distance_sub;
	Subscriber robot_blocked_sub;
	Subscriber robot_speed_sub;
	Subscriber side_sub;
	Subscriber start_sub;

	Publisher robot_status_pub;
	Publisher proximity_stop_pub;
	Publisher STM_SetPose_pub;
	Publisher STM_AsserManagement_pub;
	Publisher PANEL_Point_pub;

	ServiceClient schedulerController;

	// Robot state
	int robotState;
	bool startSignalReceived;

	int8_t direction;
	int8_t side;

	bool proximity_stop;
	bool panelUp;

	//void setRobotStatus(const RobotStatus::ConstPtr& msg);
	void setRobotPosition(const can_msgs::Point::ConstPtr& msg);
	void setRobotSpeed(const can_msgs::CurrSpeed::ConstPtr& msg);

	void processSonars(const can_msgs::SonarDistance::ConstPtr& msg);
	void onStartSignal(const ai_msgs::StartRobot& msg);

	void start();
	void stop();
};

/**
 * @}
 */
#endif
