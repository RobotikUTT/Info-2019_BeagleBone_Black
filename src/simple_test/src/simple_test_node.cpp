#include "simple_test/simple_test_node.hpp"

SimpleNode::SimpleNode(std::string name) : Node("simple_test", "none"), wentHere(true) {
    finish_sub = nh.subscribe("/ALL/Finish", 1, &SimpleNode::moveDone, this);

    this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
	this->STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
	this->STM_AsserManagement_pub = nh.advertise<can_msgs::STMStatus>("/STM/AsserManagement", 1);

    if (!this->waitForNodes(5)) {
        ROS_ERROR_STREAM("unable to start tests");
        return;
    }

    // init STM position
    can_msgs::Point msg;
    msg.pos_x = 0;
    msg.pos_y = 0;
    msg.angle = 0;
    STM_SetPose_pub.publish(msg);

    // make it running
    can_msgs::STMStatus msg2;
    msg2.value = can_msgs::STMStatus::START;
    STM_AsserManagement_pub.publish(msg2);

    ROS_INFO("Beginning movement !");
    moveSomewhereElse();
}

// Get STM signal it's done
void SimpleNode::moveDone(const can_msgs::Finish::ConstPtr& msg){
    moveSomewhereElse();
}

void SimpleNode::moveSomewhereElse() {
    ros::Duration(5).sleep();
    ROS_INFO("Changing direction !");
    
    // Revert direction
    wentHere = !wentHere;

    // Ask gently to move
    can_msgs::Point msg;
    msg.pos_x = wentHere ? 400 : 0;
    msg.pos_y = wentHere ? 10 : 580;
    msg.direction = 0;
    STMGoTo_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_test_node");

    SimpleNode moving(ros::this_node::getName());

    ros::spin();
    return 0;
}
