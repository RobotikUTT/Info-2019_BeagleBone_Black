/** @file controller_node.h
*	@brief controller_node h file
*		
*/
#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>

#include "robot_watcher/RStatus/State.h"
#include "robot_watcher/RStatus/Side.h"

#include "node_watcher/Node.hpp"

#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/SetSide.h"
#include "ai_msgs/ProximityStop.h"
#include "ai_msgs/PointsScored.h"
#include "ai_msgs/SetSchedulerState.h"

#include "can_msgs/Point.h"
#include "can_msgs/Status.h"
#include "can_msgs/SonarDistance.h"
#include "can_msgs/Frame.h"
#include "can_msgs/CurrSpeed.h"
#include "can_msgs/RobotBlocked.h"

#include "procedures_msgs/MoveAction.h"

#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"

#include "robot_interface/protocol.h"

#define FORWARD 1
#define BACKWARD -1
#define NONE 0

/**
 * @defgroup Controller The Controller package
 * @{
 */

#define SONAR_MIN_DIST_FORWARD 30 // in cm
#define SONAR_MIN_DIST_BACKWARD	10 // in cm


/**
 * @brief process inputs and change robot behavior according to that
 */
class Controller : public Node
{
public:
	Controller();

private:
	ros::Subscriber status_sub;
	ros::Subscriber robot_pos_sub;
	ros::Subscriber nodes_status_sub;
	ros::Subscriber robot_speed_sub;
	ros::Subscriber sonar_distance_sub;
	ros::Subscriber robot_blocked_sub;
	ros::Subscriber side_sub;
	ros::Subscriber start_sub;

	ros::Publisher proximity_stop_pub;
	ros::Publisher STM_SetPose_pub;
	ros::Publisher STM_AsserManagement_pub;
	ros::Publisher PANEL_Point_pub;

	ros::ServiceClient schedulerController;

	//robot pos
	int robot_pos_x;
	int robot_pos_y;
	int robot_angle;

	// Robot state
	int robot_state;
	bool startSignalReceived;

	int8_t direction;

	bool proximity_stop;
	bool side;
	bool panelUp;

	void setRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
	void setRobotPosition(const can_msgs::Point::ConstPtr& msg);
	void setRobotSpeed(const can_msgs::CurrSpeed::ConstPtr& msg);
	void setSide(const ai_msgs::SetSide::ConstPtr& msg);
	//void checkForPanel(const ai_msgs::NodesStatus::ConstPtr & msg);
	void processSonars(const can_msgs::SonarDistance::ConstPtr& msg);
	void processRobotBlocked(const can_msgs::RobotBlocked::ConstPtr& msg);
	void onStartSignal(const std_msgs::Empty& msg);

	void start();
};

/**
 * @}
 */
#endif
