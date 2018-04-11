#ifndef CAN_INTERFACE_NODE
#define CAN_INTERFACE_NODE

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <ai_msgs/RobotStatus.h>
#include <robot_watcher/Services/RobotServices.h>
#include <algorithm>

#include "robot_interface/CurrSpeed.h"
#include "robot_interface/PID.h"
#include "robot_interface/Point.h"
#include "robot_interface/PWMs.h"
#include "robot_interface/Speed.h"
#include "robot_interface/Status.h"
#include "robot_interface/STMParam.h"
#include "robot_interface/WheelsDistance.h"

#define STOP                 0
#define START                 1
#define PAUSE                 2
#define RESUME                 3
#define RESET_ID             4
#define SETEMERGENCYSTOP     5
#define NEXT_ORDER            6
#define RESET_ORDERS        7

#define HANDSHAKE    0
#define WHOAMI        1
#define SET_MODE    3
#define SPEED         4
#define GET_CODER    5
#define MANAGEMENT    7
#define GOTOA        8
#define GOTO         9
#define ROT         10
#define ROTNOMODULO 11
#define PIDLEFT        12
#define PIDRIGHT    13
#define PIDALL        14
#define PWM         15
#define SET_POS        16
#define SET_PARAM    17
#define CURRENT_POS 18
#define CURRENT_PWM    19
#define CURRENT_SPD    20

#define STM_ID 10

class CanInterfaceNode
{
public:
	CanInterfaceNode(ros::NodeHandle*);
	~CanInterfaceNode();
	void updateRobotStatus(const ai_msgs::RobotStatus::ConstPtr&);
	void canMsgProcess(const can_msgs::Frame::ConstPtr& msg);

	void STMSetMode(const robot_interface::Status::ConstPtr& msg);
	void STMSpeed(const robot_interface::Speed::ConstPtr& msg);
	void STMAsserManagement(const robot_interface::Status::ConstPtr& msg);
	void STMGoToAngle(const robot_interface::Point::ConstPtr& msg);
	void STMGoTo(const robot_interface::Point::ConstPtr& msg);
	void STMRotation(const robot_interface::Point::ConstPtr& msg);
	void STMRotationNoModulo(const robot_interface::Point::ConstPtr& msg);
	void STMLeftPID(const robot_interface::PID::ConstPtr& msg);
	void STMRightPID(const robot_interface::PID::ConstPtr& msg);
	void STMAllPID(const robot_interface::PID::ConstPtr& msg);
	void STMPWM(const robot_interface::PWMs::ConstPtr& msg);
	void STMSetPose(const robot_interface::Point::ConstPtr& msg);
	void STMSetParam(const robot_interface::STMParam::ConstPtr& msg);

private:
	ros::NodeHandle nh;
	uint8_t robot_watcher;

	ros::Publisher can_pub;

	ros::Publisher STM_coder_pub;
	ros::Publisher STM_pos_pub;
	ros::Publisher STM_pwm_pub;
	ros::Publisher STM_speed_pub;


	ros::Subscriber robot_watcher_sub;
	ros::Subscriber can_sub;

	ros::Subscriber STMSetMode_sub;
	ros::Subscriber STMSpeed_sub;
	ros::Subscriber STMAsserManagement_sub;
	ros::Subscriber STMGoToAngle_sub;
	ros::Subscriber STMGoTo_sub;
	ros::Subscriber STMRotation_sub;
	ros::Subscriber STMRotationNoModulo_sub;
	ros::Subscriber STMLeftPID_sub;
	ros::Subscriber STMRightPID_sub;
	ros::Subscriber STMAllPID_sub;
	ros::Subscriber STMPWM_sub;
	ros::Subscriber STMSetPose_sub;
	ros::Subscriber STMSetParam_sub;
};

#endif
