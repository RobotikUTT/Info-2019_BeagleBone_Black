#ifndef SIMPLE_NODE_H
#define SIMPLE_NODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <vector>

#include "node_watcher/Node.hpp"

#include "geometry_msgs/Pose2D.h"
#include "interface_msgs/StmDone.h"
#include "interface_msgs/StmMode.h"

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

    void moveDone(const interface_msgs::StmDone::ConstPtr&);
    void moveSomewhereElse();

    void onWaitingResult(bool success) override;
};

#endif
