#include "action_manager/ActionPerformer.hpp"

ActionPerformer::ActionPerformer(ros::NodeHandle* n, std::string name) : name(name) {
    this->nh = *n;

    // Service for action point computation
    std::ostringstream srvName("action_manager/actionPoint/");
    srvName << name;
    actionPointSrv  = nh.advertiseService(srvName.str(), &ActionPerformer::_computeActionPoint, this);

    // Action server for this action
    std::ostringstream actionName("action_");
    actionName << name;
    actionServer = new PerformActionSrv(actionName.str(), false);

    actionServer->registerGoalCallback(boost::bind(&ActionPerformer::onGoal, this));
    actionServer->registerPreemptCallback(boost::bind(&ActionPerformer::onPreempt, this));
    actionServer->start();
}

bool ActionPerformer::_computeActionPoint(ai_msgs::ComputeActionPoint::Request& req, ai_msgs::ComputeActionPoint::Response& res) {
    // Extract data from request and call performer function
    ActionPoint* result = computeActionPoint(req.args, req.robot_pos);

    // Copy result to response
    res.start_point.x = result->startPoint.x;
    res.start_point.y = result->startPoint.y;
    res.end_point.x = result->endPoint.x;
    res.end_point.y = result->endPoint.y;
}

void ActionPerformer::onGoal() {

}

void ActionPerformer::onPreempt() {

}