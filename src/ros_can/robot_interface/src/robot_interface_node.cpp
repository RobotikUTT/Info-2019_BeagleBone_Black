#include <robot_interface/robot_interface_node.h>
#include<iterator>


CanInterfaceNode::CanInterfaceNode(ros::NodeHandle *n){
	this->nh = *n;

	this->can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);
	this->test_pub = nh.advertise<sender::test2>("receiver/test", 1000);

	this->robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_watcher", 1000, &CanInterfaceNode::updateRobotStatus, this);
	this->test_sub = nh.subscribe("/ros_can/interface/test", 1000, &CanInterfaceNode::test, this);
	this->can_sub = nh.subscribe("received_messages", 1000, &CanInterfaceNode::canMsgProcess, this);

	service_ready("ros_can", "interface", 1 );

}

CanInterfaceNode::~CanInterfaceNode(){

}

void CanInterfaceNode::updateRobotStatus(const robot_watcher::RobotStatus::ConstPtr& msg){
	this->robot_watcher = msg->robot_watcher;
	// ROS_INFO("callback robot_watcher: %d", this->robot_watcher);
}

void CanInterfaceNode::test(const sender::test2::ConstPtr& msg){
	// ROS_INFO("callback mode: %d", msg->mode);
	// for (int i = 0; i < 7; ++i)
	// {
	// 	ROS_INFO("callback data %d: %d",i, msg->data[i]);
	// }
	can_msgs::Frame fr;
	fr.header.stamp = ros::Time::now();
	fr.header.frame_id = "/ros_can/interface/";
	fr.id = 1;
	fr.is_rtr = 0;
	fr.is_error = 0;
	fr.is_extended = 0;
	fr.dlc = 8;
	fr.data[0] = msg->mode;
	std::copy(std::begin(msg->data), std::end(msg->data), std::next(std::begin(fr.data),1));

	can_pub.publish(fr);
}

void CanInterfaceNode::canMsgProcess(const can_msgs::Frame::ConstPtr& msg){
	sender::test2 te;

	if (msg->dlc != 0)
	{
		te.mode = msg->data[0];
		std::copy(std::next(std::begin(msg->data), 1) , std::next(std::begin(msg->data),msg->dlc ), std::begin(te.data) );
	}


	test_pub.publish(te);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "can_node");
	 
	ros::NodeHandle nmh;

	CanInterfaceNode node (&nmh);

	ros::spin();
}
