#include "controller/controller_node.h"


Controller::Controller(ros::NodeHandle* n):
acM(nh, "move", true){
  nh = *n;

  status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Scheduler::setSide, this);
  //sub for sonar data
  robot_pos_sub = nh.subscribe("/STM/Position", 1, &Scheduler::GetRobotStatus, this)

  nmh.getParam("~robot_pos/x", robot_pos_x);
  nmh.getParam("~robot_pos/y", robot_pos_y);

  client = nh.serviceClient<ai_msgs::GetActionToDo>("scheduler/actionToDo");
  client.waitForExistence();


  service_ready("ai", "controller", 1 );

}

//function urgency stop

//function to get robot pos
void Controller::GetActionToDo(const can_msgs::Point::ConstPtr & msg){
  robot_pos_x = msg->pos_x;
  robot_pos_y = msg->pos_y;
}

void Controller::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  robot_status = msg->robot_watcher;

  if(robot_status == RobotState.ROBOT_READY){
    //init card

    //get actionToDo
    ai_msgs::GetActionToDo srv;
    srv.request.robot_pos_x = robot_pos_x;
    srv.request.robot_pos_y = robot_pos_y;
    if (client.call(srv)) {
      action_name = srv.response.action_name;
    } else {
      ROS_ERROR("Failed to call service GetActionToDo");
    }
  } else if (robot_status == RobotState.ROBOT_RUNNING) {
    //set action to true
    //init pos
  }
}

template <class doneMsg>
void Controller::DoneAction( const actionlib::SimpleClientGoalState& state,
                        const doneMsg & result){
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  //desactivate old action

  //send point


  //ask for new action

  //activate new action

}

// ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

void Controller::activeAction(){
  ROS_INFO("Goal just went active");
}

void Controller::feedbackCb(const procedures_msgs::XXFeedback& feedback){
  ROS_INFO("Feedback");
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "controller_node");

	ros::NodeHandle nmh;

  Controller node (&nmh);

	ros::spin();
}
