#ifndef ACTION_PERFORMER_H
#define ACTION_PERFORMER_H

#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include "ai_msgs/PerformAction.h"

#include "procedures_msgs/OrPoint.h"

typedef actionlib::SimpleActionServer<ai_msgs::PerformAction> PerfomActionSrv;

/**
 * Represent an performer for a specific action, it advertise a service for
 * predicting position and an action server to run the action.
 * 
 * It provide simple interface for creating robot action, and avoid tedious ROS objects
 * manipulation.
 */
class ActionPerformer
{
public:
    ActionPerformer(std::string name);

    // Function defined by inherited actions
    virtual ActionPoint* computeActionPoint(Argument[] args, OrPoint robot_pos);
    virtual run(Argument[] args);

    // Call adequate service to retrieve the action point
    static ActionPoint* computeActionPoint(std::string performer);
private:
    bool _computeActionPoint(ai_msgs::ComputeActionPoint::Request &req, ai_msgs::ComputeActionPoint::Response &res)

    std::string name;
    PerformActionSrv* actionServer;
};

#endif
