#include <ros_can_interface/ros_can_interface_node.h>


CanInterfaceNode::CanInterfaceNode(ros::NodeHandle *n){
	this->nh = *n;

	this->can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);

	this->robot_status_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1000, &CanInterfaceNode::updateRobotStatus, this);

}

CanInterfaceNode::~CanInterfaceNode(){

}

void CanInterfaceNode::updateRobotStatus(const ai_robot_status::RobotStatus::ConstPtr& msg){
	this->robot_status = msg->robot_status;
	// ROS_INFO("callback robot_status: %d", this->robot_status);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "can_node");
	
	ros::NodeHandle nmh;

	CanInterfaceNode node (&nmh);

	ros::spin();
}
