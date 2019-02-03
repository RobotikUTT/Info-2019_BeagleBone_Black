/** @file ActionClass.cpp
*    @brief Class for ActionClass.
*    
*    
*    @author Alexis CARE
*/
#include "scheduler/scheduler_node.hpp"

/**
 * @brief      Constructs the object.
 *
 * @param      n     NodeHandler
 */
Scheduler::Scheduler(ros::NodeHandle* n){
  this->nh = *n;

  std::string actions_file;
  nh.getParam("scheduler/config_file", actions_file);

  this->side = SIDE_GREEN;

  this->side_sub = nh.subscribe("side", 1, &Scheduler::setSide, this);

  this->action_srv = nh.advertiseService("scheduler/actionToDo", &Scheduler::getActionToDo, this);
  this->actionD_srv = nh.advertiseService("scheduler/currentActionDone", &Scheduler::currentActionDone, this);
  
  //this-> = ActionManager(actions_file.c_str());

  ActionsParser parser(actions_file);

  service_ready("ai", "scheduler", 1 );
}

/**
 * @brief      Sets the starting side.
 *
 * @param[in]  msg   The SetSide message
 */
void Scheduler::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  if(this->side != msg->side){
    this->side = ! this->side;
    // TODO acm
    // this->actionManager.changeSide();
  }
}

/**
 * @brief      Gets the action to exectute.
 *
 * @param      req   The service message request
 * @param      res   The service message resource
 *
 */
bool Scheduler::getActionToDo(ai_msgs::GetActionToDo::Request &req, ai_msgs::GetActionToDo::Response &res){
  // ROS_INFO_STREAM("action position x: " << req.robot_pos_x);
  // ROS_INFO_STREAM("action position y: " << req.robot_pos_y);

  // TODO acm
  //this->actionManager.updatePriority(Point(req.robot_pos_x,req.robot_pos_y));
  //this->actionManager.getActionToDo(res);
  return true;
}

/**
 * @brief      Set done's attribut Action 
 *
 * @param      req   The service message request
 * @param      res   The service message resource
 *
 */
bool Scheduler::currentActionDone(ai_msgs::CurrentActionDone::Request &req, ai_msgs::CurrentActionDone::Response &res){
  // TODO acm
  //this->actionManager.currentActionDone(req.done);
  return true;
}


int main(int argc, char *argv[]) {
  ros::init(argc,argv, "scheduler_node");

	ros::NodeHandle nmh;

  Scheduler node (&nmh);

	ros::spin();
}
