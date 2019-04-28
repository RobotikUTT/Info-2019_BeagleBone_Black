#include "simple_test/simple_test_node.hpp"

SimpleNode::SimpleNode(std::string name) : Node("simple_test", "none"), wentHere(true) {
    finish_sub = nh.subscribe("/ALL/Finish", 1, &SimpleNode::moveDone, this);

    this->STMGoTo_pub = nh.advertise<geometry_msgs::Pose2D>("/STM/GoTo", 1);
	this->STM_SetPose_pub = nh.advertise<geometry_msgs::Pose2D>("/STM/SetPose", 1);
	this->STM_AsserManagement_pub = nh.advertise<interface_msgs::StmMode>("/STM/AsserManagement", 1);

    this->waitForNodes(5);
}

void SimpleNode::onWaitingResult(bool success) {
	if (success) {
		// init STM position
        geometry_msgs::Pose2D msg;
        msg.x = 0;
        msg.y = 0;
        msg.theta = 0;
        STM_SetPose_pub.publish(msg);

        // make it running
        interface_msgs::StmMode msg2;
        msg2.value = interface_msgs::StmMode::START;
        STM_AsserManagement_pub.publish(msg2);

        ROS_INFO("Beginning movement !");
        moveSomewhereElse();
	} else {
		ROS_ERROR_STREAM("unable to start tests");
        return;
	}
}

// Get STM signal it's done
void SimpleNode::moveDone(const interface_msgs::StmDone::ConstPtr& msg){
    moveSomewhereElse();
}

void SimpleNode::moveSomewhereElse() {
    ros::Duration(5).sleep();
    ROS_INFO("Changing direction !");
    
    // Revert direction
    wentHere = !wentHere;

    // Ask gently to move
    geometry_msgs::Pose2D msg;
    msg.x = wentHere ? 400 : 0;
    msg.y = wentHere ? 10 : 580;
    STMGoTo_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_test_node");

    SimpleNode moving(ros::this_node::getName());

    ros::spin();
    return 0;
}
