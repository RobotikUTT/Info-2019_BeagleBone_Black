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
#include "action_manager/Action.hpp"

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
    ActionPerformer(std::string name);

    // Call adequate service to retrieve the action point
    static ActionPoint* computeActionPoint(std::string performer);

protected:
    // ROS nodehandle is protected to let child use ros
    ros::NodeHandle nh;

    // Function defined by inherited actions
    virtual ActionPoint* computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, procedures_msgs::OrPoint& robot_pos);
    virtual void start();
    virtual void cancel();

    double getArg(std::string name, double defaultValue = 0, std::vector<ai_msgs::Argument>* args = NULL);

    // Function managing the action
    void actionPerformed();
    void actionPaused();
private:
    std::vector<ai_msgs::Argument> _args;

    bool _computeActionPoint(
        ai_msgs::ComputeActionPoint::Request& req,
        ai_msgs::ComputeActionPoint::Response& res
    );

    void onGoal(const ai_msgs::PerformActionGoalConstPtr& goal, PerformActionSrv* as);
    void onPreempt();

    // Name of the perfomer
    std::string name;

    // Ros objects
    PerformActionSrv* actionServer;
    ros::ServiceServer actionPointSrv;
};

#endif
