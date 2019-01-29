#ifndef ACTION_PERFORMER_H
#define ACTION_PERFORMER_H

#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include "ai_msgs/PerformAction.h"

typedef actionlib::SimpleActionServer<ai_msgs::PerformAction> PerfomActionSrv;

class ActionPerformer
{
public:
    ActionPerformer(std::string name);
private:
    std::string name;
};

#endif
