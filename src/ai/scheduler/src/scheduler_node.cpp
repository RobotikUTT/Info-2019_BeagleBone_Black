#include "scheduler/scheduler_node.h"


Scheduler::Scheduler(ros::NodeHandle* n){
  this->nh = *n;

  this->side = SIDE_GREEN;

  this->side_sub = nh.subscribe("ai/side", 1, &Scheduler::setSide, this);

  this->actionManager = ActionManager();

  service_ready("ai", "scheduler", 1 );

}

void Scheduler::setSide(const robot_watcher::SetSide::ConstPtr& msg){
  if(this->side != msg->side){
    this->side = ! this->side;
    this->actionManager.changeSide();
  }
}


int main(int argc, char *argv[]) {
  ros::init(argc,argv, "scheduler_node");

	ros::NodeHandle nmh;

  Scheduler node (&nmh);

	ros::spin();
}
