#include "controller/controller_node.h"


Controller::Controller(ros::NodeHandle* n){
  nh = *n;

  status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Scheduler::setSide, this);

  client = nh.serviceClient<ai_msgs::GetActionToDo>("scheduler/actionToDo");
  client.waitForExistence();

  service_ready("ai", "controller", 1 );

}

Controller::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  robot_status = msg->robot_watcher;
}


int main(int argc, char *argv[]) {
  ros::init(argc,argv, "controller_node");

	ros::NodeHandle nmh;

  Controller node (&nmh);

	ros::spin();
}
