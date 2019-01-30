#include "action_manager/ActionPerformer.hpp"

ActionPerformer::ActionPerformer(std::string name) : name(name) {
    // Service for action point computation
    std::ostringstream srvName("action_manager/actionPoint/");
    srvName << name;
    this->action_point_srv  = nh.advertiseService(srvName.str(), &ActionPerformer::_computeActionPoint, this);

    // Action server for this action
    std::ostringstream actionName("action_");
    actionName << name;
    actionServer = new PerfomActionSrv(actionName.str());

    actionServer.registerGoalCallback(boost::bind(&Move::goalCB, this));
    actionServer.registerPreemptCallback(boost::bind(&Move::preemptCB, this));
    actionServer.start();
}

bool ActionPerformer::_computeActionPoint(ai_msgs::ComputeActionPoint::Request &req, ai_msgs::ComputeActionPoint::Response &res) {
    // Extract data from request and call performer function
    ActionPoint* result = computeActionPoint(req.args, req.robot_pos);

    // Copy result to response
    res.start_point.x = result->startPoint.x;
    res.start_point.y = result->startPoint.y;
    res.end_point.x = result->endPoint.x;
    res.end_point.y = result->endPoint.y;
}