#ifndef CAN_INTERFACE_NODE
#define CAN_INTERFACE_NODE

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <ai_robot_status/RobotStatus.h>

class CanInterfaceNode
{
public:
	CanInterfaceNode(ros::NodeHandle*);
	~CanInterfaceNode();
	void updateRobotStatus(const ai_robot_status::RobotStatus::ConstPtr&);
private:
	ros::NodeHandle nh;
	uint8_t robot_status;

	ros::Publisher can_pub;

	ros::Subscriber robot_status_sub;
};

#endif