#ifndef CAN_INTERFACE_NODE
#define CAN_INTERFACE_NODE

#include <ros/ros.h>
#include <iterator>
#include <algorithm>

#include <robot_watcher/Services/RobotServices.h>

#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/NodeStatus.h"

#include "can_msgs/Frame.h"

#include "interface_msgs/WheelsSpeed.h"
#include "interface_msgs/StmDone.h"
#include "interface_msgs/ObjectOnMap.h"
#include "interface_msgs/Pid.h"
#include "interface_msgs/Pwms.h"
#include "interface_msgs/Point.h"
#include "interface_msgs/RobotBlocked.h"
#include "interface_msgs/StmParams.h"
#include "interface_msgs/SonarDistance.h"
#include "interface_msgs/Speed.h"
#include "interface_msgs/StmMode.h"
#include "interface_msgs/WheelsDistance.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Int8.h"

#include "node_watcher/Node.hpp"


using can_msgs::Frame;

/**
 * @defgroup Robot_Interface The robot_interface package
 * @{
 */

/**
 * @brief      Class for can interface node.
 */
class CanInterfaceNode
{
public:
	CanInterfaceNode(ros::NodeHandle*);
	~CanInterfaceNode();
	void updateRobotStatus		(const ai_msgs::RobotStatus::ConstPtr&);
	void canMsgProcess			(const can_msgs::Frame::ConstPtr& msg);

	void ALLPing				(const std_msgs::Empty::ConstPtr& msg);
	void STMSetMode				(const interface_msgs::StmMode::ConstPtr& msg);
	void STMSpeed				(const interface_msgs::Speed::ConstPtr& msg);
	void STMAsserManagement		(const interface_msgs::StmMode::ConstPtr& msg);
	void STMGoToAngle			(const interface_msgs::Point::ConstPtr& msg);
	void STMGoTo				(const interface_msgs::Point::ConstPtr& msg);
	void STMRotation			(const interface_msgs::Point::ConstPtr& msg);
	void STMRotationNoModulo	(const interface_msgs::Point::ConstPtr& msg);
	void STMLeftPid				(const interface_msgs::Pid::ConstPtr& msg);
	void STMRightPid			(const interface_msgs::Pid::ConstPtr& msg);
	void STMAllPid				(const interface_msgs::Pid::ConstPtr& msg);
	void STMPWM					(const interface_msgs::Pwms::ConstPtr& msg);
	void STMSetPose				(const interface_msgs::Point::ConstPtr& msg);
	void STMSetParam			(const interface_msgs::StmParams::ConstPtr& msg);
	void PANELAddPoint			(const std_msgs::Int8::ConstPtr& msg);

private:
	ros::NodeHandle nh;
	uint8_t robot_watcher;

	ros::Publisher can_pub;

	ros::Publisher STM_coder_pub;
	ros::Publisher STM_pos_pub;
	ros::Publisher STM_pwm_pub;
	ros::Publisher STM_speed_pub;
	ros::Publisher ALL_finish_pub;
	ros::Publisher ARDUINO_sonar_distance_pub;
	ros::Publisher STM_robot_blocked_pub;
	// ros::Publisher LIDAR_objec_on_map_pub;

	ros::Subscriber robot_watcher_sub;
	ros::Subscriber can_sub;

	ros::Subscriber ALL_Ping_sub;
	ros::Subscriber STM_SetMode_sub;
	ros::Subscriber STM_Speed_sub;
	ros::Subscriber STM_AsserManagement_sub;
	ros::Subscriber STM_GoToAngle_sub;
	ros::Subscriber STM_GoTo_sub;
	ros::Subscriber STM_Rotation_sub;
	ros::Subscriber STM_LeftPid_sub;
	ros::Subscriber STM_RightPid_sub;
	ros::Subscriber STM_AllPid_sub;
	ros::Subscriber STM_PWM_sub;
	ros::Subscriber STM_SetPose_sub;
	ros::Subscriber STM_SetParam_sub;
	ros::Subscriber PANEL_point_sub;
};

/**
 * @}
 */
#endif
