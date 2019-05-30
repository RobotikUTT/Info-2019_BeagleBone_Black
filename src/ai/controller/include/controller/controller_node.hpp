/** @file controller_node.h
*	@brief controller_node h file
*		
*/
#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include <ros/ros.h>

#include "node_watcher/Node.hpp"

#include "args_lib/Argumentable.hpp"

#include "ai_msgs/NodeStatus.h"
#include "ai_msgs/Side.h"

#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/ProximityStop.h"
#include "ai_msgs/StartRobot.h"
#include "ai_msgs/PointsScored.h"
#include "ai_msgs/SetSchedulerState.h"
#include "ai_msgs/GetSidedPoint.h"

#include "geometry_msgs/Pose2D.h"

#include "interface_msgs/CanData.h"
#include "interface_msgs/Directions.h"
#include "interface_msgs/StmMode.h"

#include "std_msgs/Int8.h"

#include <math.h>

/**
 * @defgroup Controller The Controller package
 * @{
 */

#define SONAR_MIN_DIST_FORWARD 25 // in cm
#define SONAR_MIN_DIST_BACKWARD	25 // in cm

using ros::Subscriber;
using ros::Publisher;
using ros::ServiceClient;

// Use ai_msgs namespace to simplify usage
using namespace ai_msgs;

using interface_msgs::Directions;

/**
 * @brief process inputs and change robot behavior according to that
 */
class Controller : public Node
{
public:
	Controller();

private:
	ros::Timer timer;

	Subscriber status_sub;
	Subscriber nodes_status_sub;
	Subscriber can_data_sub;
	Subscriber start_sub;

	Publisher can_data_pub;

	ServiceClient schedulerController;
	ServiceClient sidedPoint;

	// Robot state
	int robotState;
	bool startSignalReceived;
	bool done;

	int8_t direction;
	int8_t side;

	bool proximity_stop;
	bool panelUp;

	//void setRobotStatus(const RobotStatus::ConstPtr& msg);
	void setRobotPosition(const geometry_msgs::Pose2D::ConstPtr& msg);
	void onCanData(const interface_msgs::CanData::ConstPtr& msg);

 	void processSonars(const Argumentable& data);

	void onStartSignal(const ai_msgs::StartRobot& msg);

	void start();
	void stop(const ros::TimerEvent& timer);

	void onWaitingResult(bool success) override;
};

/**
 * @}
 */
#endif
