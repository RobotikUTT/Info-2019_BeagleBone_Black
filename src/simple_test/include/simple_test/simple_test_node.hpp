#ifndef SIMPLE_NODE_H
#define SIMPLE_NODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <vector>

#include "node_watcher/Node.hpp"

#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"
#include "can_msgs/STMStatus.h"

#include "ai_msgs/NodeRequirement.h"

class SimpleNode : public Node {
public:
    SimpleNode(std::string name);

private:
    ros::NodeHandle nh;

    ros::Subscriber finish_sub;
    ros::Publisher STMGoTo_pub;
    ros::Publisher STM_SetPose_pub;
    ros::Publisher STM_AsserManagement_pub;

    bool wentHere;

    void moveDone(const can_msgs::Finish::ConstPtr&);
    void moveSomewhereElse();
};

#endif
