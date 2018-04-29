#ifndef CAN_INTERFACE_NODE
#define CAN_INTERFACE_NODE

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <ai_msgs/RobotStatus.h>
#include <robot_watcher/Services/RobotServices.h>
#include <algorithm>
#include "robot_interface/protocol.h"

#include "can_msgs/ActionPliers.h"
#include "can_msgs/CurrSpeed.h"
#include "can_msgs/Finish.h"
#include "can_msgs/ObjectOnMap.h"
#include "can_msgs/PID.h"
#include "can_msgs/PWMs.h"
#include "can_msgs/Point.h"
#include "can_msgs/RobotBlocked.h"
#include "can_msgs/STMParam.h"
#include "can_msgs/SonarDistance.h"
#include "can_msgs/Speed.h"
#include "can_msgs/Status.h"
#include "can_msgs/ThrowBalls.h"
#include "can_msgs/WheelsDistance.h"
#include "can_msgs/ActionPliers.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Int8.h"

#define ALL_CAN_ADDR            0
#define BBB_CAN_ADDR            1
#define STM_CAN_ADDR            2
#define ARDUINO_CAN_ADDR        3
#define ZIGBEE_CAN_ADDR         4
#define PANEL_CAN_ADDR          5


#define HANDSHAKE               0
#define WHOAMI                  1
#define SET_MODE                2

#define SPD                     4
#define GET_CODER               5
#define SEND_POINT              6
#define MANAGEMENT              7
#define GOTOA                   8
#define GOTO                    9
#define ROT                     10
#define ROTNOMODULO             11
#define PIDLEFT                 12
#define PIDRIGHT                13
#define PIDALL                  14
#define PWM                     15
#define SET_POS                 16
#define SET_PARAM               17
#define CURRENT_POS             18
#define CURRENT_PWM             19
#define CURRENT_SPD             20
#define MOVE_PLIERS             21
#define CLOSE_OPEN_PLIERS       22
#define SONAR_DISTANCE          23
#define THROW_BALLS             24
#define OBJECT_ON_MAP           25
#define ORDER_COMPLETED         26
#define SET_SERVO               27
#define ROBOT_BLOCKED           28
#define ACTION_PLIERS           29

class CanInterfaceNode
{
public:
	CanInterfaceNode(ros::NodeHandle*);
	~CanInterfaceNode();
	void updateRobotStatus(const ai_msgs::RobotStatus::ConstPtr&);
	void canMsgProcess(const can_msgs::Frame::ConstPtr& msg);

	void ALLPing(const std_msgs::Empty::ConstPtr& msg);
	void STMSetMode(const can_msgs::Status::ConstPtr& msg);
	void STMSpeed(const can_msgs::Speed::ConstPtr& msg);
	void STMAsserManagement(const can_msgs::Status::ConstPtr& msg);
	void STMGoToAngle(const can_msgs::Point::ConstPtr& msg);
	void STMGoTo(const can_msgs::Point::ConstPtr& msg);
	void STMRotation(const can_msgs::Point::ConstPtr& msg);
	void STMRotationNoModulo(const can_msgs::Point::ConstPtr& msg);
	void STMLeftPID(const can_msgs::PID::ConstPtr& msg);
	void STMRightPID(const can_msgs::PID::ConstPtr& msg);
	void STMAllPID(const can_msgs::PID::ConstPtr& msg);
	void STMPWM(const can_msgs::PWMs::ConstPtr& msg);
	void STMSetPose(const can_msgs::Point::ConstPtr& msg);
	void STMSetParam(const can_msgs::STMParam::ConstPtr& msg);
	void ARDUINOThrowBalls(const can_msgs::ThrowBalls::ConstPtr& msg);
	void ARDUINOActionPliers(const can_msgs::ActionPliers::ConstPtr& msg);
	void PANELAddPoint(const std_msgs::Int8::ConstPtr& msg);

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
	ros::Subscriber STM_RotationNoModulo_sub;
	ros::Subscriber STM_LeftPID_sub;
	ros::Subscriber STM_RightPID_sub;
	ros::Subscriber STM_AllPID_sub;
	ros::Subscriber STM_PWM_sub;
	ros::Subscriber STM_SetPose_sub;
	ros::Subscriber STM_SetParam_sub;
	ros::Subscriber ARDUINO_ActionPliers_sub;
	ros::Subscriber ARDUINO_ThrowBalls_sub;
	ros::Subscriber PANEL_point_sub;
};

#endif
