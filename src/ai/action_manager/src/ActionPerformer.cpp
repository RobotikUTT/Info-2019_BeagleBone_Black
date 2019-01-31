#include "action_manager/ActionPerformer.hpp"

// TODO take robot status into consideration to handle ROBOT_HALT

ActionPerformer::ActionPerformer( std::string name) : name(name), nh() {
    // Service for action point computation
    std::ostringstream srvName("action_manager/actionPoint/");
    srvName << name;
    actionPointSrv  = nh.advertiseService(srvName.str(), &ActionPerformer::_computeActionPoint, this);

    // Action server for this action
    std::ostringstream actionName("action_");
    actionName << name;
    actionServer = new PerformActionSrv(actionName.str(), boost::bind(&ActionPerformer::onGoal, this), false);

    //actionServer->registerGoalCallback(boost::bind(&ActionPerformer::onGoal, this));
    actionServer->registerPreemptCallback(boost::bind(&ActionPerformer::onPreempt, this));
    actionServer->start();
}

bool ActionPerformer::_computeActionPoint(ai_msgs::ComputeActionPoint::Request& req, ai_msgs::ComputeActionPoint::Response& res) {
    // Extract data from request and call performer function
    ActionPoint* result = computeActionPoint(&req.args, req.robot_pos);

    // Copy result to response
    res.start_point.x = result->startPoint.x;
    res.start_point.y = result->startPoint.y;
    res.end_point.x = result->endPoint.x;
    res.end_point.y = result->endPoint.y;
}

/**
 *  Goal received
 */
void ActionPerformer::onGoal(const ai_msgs::PerformActionGoalConstPtr& goal, PerformActionSrv* as) {
    //ai_msgs::PerformActionGoal::ConstPtr goal = actionServer->acceptNewGoal();
    _args = goal->args;

    start();
}

/**
 *  Preempt received
 */
void ActionPerformer::onPreempt() {
    // Cancel current goal if needed
    cancel();
    
    // Mark as preempted
    actionServer->setPreempted();
}

/**
 * Terminate the action
 */
void ActionPerformer::actionPerformed() {
    // Create result message
    ai_msgs::PerformActionResult result;
    result.status = ACTION_DONE;

    // Send back to client
    actionServer->setSucceeded(result);
}

/**
 * Pause the action
 */
void ActionPerformer::actionPaused() {
    // Create result message
    ai_msgs::PerformActionResult result;
    result.status = ACTION_PAUSED;

    // Send back to client
    actionServer->setSucceeded(result);
}

/**
 *  Retrieve an argument from the argument list
 */
double ActionPerformer::getArg(std::string name, double defaultValue /*= 0*/, std::vector<ai_msgs::Argument>* args /*= NULL*/) {
    // No arg list provided -> use current action's one
    if (args == NULL) {
        args = &_args;
    }

    // Seek an argument with this name
    for (const auto& next : args) {
        if (name.compare(next.name) == 0) {
            return next.value;
        }
    }

    return defaultValue;
}

// Virtual functions
ActionPoint* ActionPerformer::computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, procedures_msgs::OrPoint& robot_pos) {
    return new ActionPoint();
}
void ActionPerformer::start() { }
void cancel() {};