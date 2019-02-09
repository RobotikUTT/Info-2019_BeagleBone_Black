#include "simple_test/simple_test_node.hpp"

SimpleNode::SimpleNode(std::string name) : wentHere(true) {
    finish_sub = nh.subscribe("/ALL/Finish", 1, &SimpleNode::moveDone, this);

    this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
    this->STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);

    moveSomewhereElse();
}

// Get STM signal it's done
void SimpleNode::moveDone(const can_msgs::Finish::ConstPtr& msg){
    moveSomewhereElse();
}

void SimpleNode::moveSomewhereElse() {
    ros::Duration(5).sleep();
    ROS_INFO("Start moving !");
    
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
