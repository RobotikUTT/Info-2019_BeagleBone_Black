#include "scheduler/scheduler_node.h"


Scheduler::Scheduler(ros::NodeHandle* n){
  this->nh = *n;

  this->side = SIDE_GREEN;

  this->side_sub = nh.subscribe("side", 1, &Scheduler::setSide, this);

  this->action_srv = nh.advertiseService("actionToDo", &Scheduler::getActionToDo, this);
  this->actionManager = ActionManager();

  service_ready("ai", "scheduler", 1 );

}

void Scheduler::setSide(const robot_watcher::SetSide::ConstPtr& msg){
  if(this->side != msg->side){
    this->side = ! this->side;
    this->actionManager.changeSide();
  }
}

bool Scheduler::getActionToDo(robot_watcher::GetActionToDo::Request &req, robot_watcher::GetActionToDo::Response &res){
  this->actionManager.updatePriority(Point(req.robot_pos_x,req.robot_pos_y));
  res.action_name = this->actionManager.getActionToDo();
  return true;
}


int main(int argc, char *argv[]) {
  ros::init(argc,argv, "scheduler_node");

	ros::NodeHandle nmh;

  Scheduler node (&nmh);

	ros::spin();
}
