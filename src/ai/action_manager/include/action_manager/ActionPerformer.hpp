#ifndef ACTION_PERFORMER_H
#define ACTION_PERFORMER_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "ai_msgs/PerformAction.h"
#include "ai_msgs/Argument.h"
#include "ai_msgs/ComputeActionPoint.h"
#include "action_manager/ActionPoint.h"
#include "procedures_msgs/OrPoint.h"

typedef actionlib::SimpleActionServer<ai_msgs::PerformAction> PerformActionSrv;

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
    ActionPerformer(ros::NodeHandle* n, std::string name);

    // Function defined by inherited actions
    virtual ActionPoint* computeActionPoint(std::vector<ai_msgs::Argument> actionArgs, procedures_msgs::OrPoint robot_pos);
    virtual void run(std::vector<ai_msgs::Argument> actionArgs);

    // Call adequate service to retrieve the action point
    static ActionPoint* computeActionPoint(std::string performer);
private:
    bool _computeActionPoint(
        ai_msgs::ComputeActionPoint::Request& req,
        ai_msgs::ComputeActionPoint::Response& res
    );

    void onGoal();
    void onPreempt();

    // Name of the perfomer
    std::string name;

    // Ros objects
    ros::NodeHandle nh;

    PerformActionSrv* actionServer;
    ros::ServiceServer actionPointSrv;
};

#endif
