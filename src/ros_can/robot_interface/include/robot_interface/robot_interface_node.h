#ifndef CAN_INTERFACE_NODE
#define CAN_INTERFACE_NODE

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <robot_watcher/RobotStatus.h>
#include <robot_watcher/Services/RobotServices.h>
#include <sender/test2.h>
#include <algorithm>

class CanInterfaceNode
{
public:
	CanInterfaceNode(ros::NodeHandle*);
	~CanInterfaceNode();
	void updateRobotStatus(const robot_watcher::RobotStatus::ConstPtr&);
	void test(const sender::test2::ConstPtr& msg);
	void canMsgProcess(const can_msgs::Frame::ConstPtr& msg);
private:
	ros::NodeHandle nh;
	uint8_t robot_watcher;

	ros::Publisher can_pub;
	ros::Publisher test_pub;

	ros::Subscriber robot_watcher_sub;
	ros::Subscriber test_sub;
	ros::Subscriber can_sub;
};

#endif