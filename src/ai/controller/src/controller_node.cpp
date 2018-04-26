#include "controller/controller_node.h"

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const> MoveResultConstPtr;

Controller::Controller(ros::NodeHandle* n):
acM("/procedures/move_action", true){
  nh = *n;

  status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Controller::GetRobotStatus, this);
  //sub for sonar data
  //sonar_sub = nh.subscribe("/Arduino/sonars", 1, &Controller::GetSonars, this);
  robot_pos_sub = nh.subscribe("/STM/Position", 1, &Controller::GetRobotPose, this);
  side_sub = nh.subscribe("side", 1, &Controller::setSide, this);

  // emergency_stop_pub = nh.advertise<ai_msgs::EmergencyStop>("emergency", 1);
  STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
  STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);

  side = SIDE_GREEN;

  clientD = nh.serviceClient<ai_msgs::CurrentActionDone>("scheduler/currentActionDone");
  clientA = nh.serviceClient<ai_msgs::GetActionToDo>("scheduler/actionToDo");
  clientA.waitForExistence();

  if(!acM.waitForServer(ros::Duration(10))){
    ROS_ERROR("NOPE");
  } else {
    service_ready("ai", "controller", 1 );
  }

}

//function urgency stop
// void Controller::GetSonars(const can_msgs::Sonars::ConstPtr & msg) {
//   /* code */
// }

void Controller::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

//function to get robot pos
void Controller::GetRobotPose(const can_msgs::Point::ConstPtr & msg){
  robot_pos_x = msg->pos_x;
  robot_pos_y = msg->pos_y;
  robot_angle = msg->angle;
  // compute direction
}

void Controller::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  robot_status = msg->robot_watcher;

  if(robot_status == ROBOT_READY){
    //init card
    // ROS_DEBUG("Robot Ready");


  } else if (robot_status == ROBOT_RUNNING) {
    // ROS_DEBUG("Robot Running");

      if(side){
        robot_pos_x = (1500 - robot_pos_x) + 1500;
        robot_angle = -robot_angle;
      }
      //init pos STM
      can_msgs::Point msg;
      int x,y,angle;
      nh.getParam("controller/robot_pos/x", x);
      nh.getParam("controller/robot_pos/y", y);
      nh.getParam("controller/robot_pos/angle", angle);

      robot_pos_x = x;
      robot_pos_y = y;
      robot_angle = angle;

      msg.pos_x = x;
      msg.pos_y = y;
      msg.angle = angle;

      STM_SetPose_pub.publish(msg);
      can_msgs::Status msg2;
      msg2.value = 1;
      STM_AsserManagement_pub.publish(msg2);
      SetAction();
      //set action to true
  }
}

template <class doneMsg>
void Controller::DoneAction( const actionlib::SimpleClientGoalState& state,
                        const doneMsg & result){
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  //desactivate old action
  // acM.shutdown()

  //send point
  points_done += result->points_done;


  //set old action to done
  ai_msgs::CurrentActionDone srv;
  srv.request.done = result->done;
  // ROS_INFO_STREAM("Action in state " << result->done);
  if (!clientD.call(srv)) {
    ROS_ERROR("Failed to call service currentActionDone");
  }

  //ask for new action
  //activate new action
  SetAction();

}

// ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

// void Controller::activeAction(){
//   ROS_INFO("Goal just went active");
// }

// void Controller::feedbackCb(const procedures_msgs::XXFeedback& feedback){
//   ROS_INFO("Feedback");
// }

void Controller::SetAction(){
  ai_msgs::GetActionToDo srv;
  srv.request.robot_pos_x = robot_pos_x;
  srv.request.robot_pos_y = robot_pos_y;
  if (clientA.call(srv)) {
    action_val = srv.response.action_val;
    // ROS_DEBUG("ACtion value: %d", action_val);
  } else {
    ROS_ERROR("Failed to call service GetActionToDo");
    action_val = -1;
  }

  if(action_val == MOVE)
  {
    ROS_WARN("Action MOVE");
    procedures_msgs::MoveGoal goal;
    goal.points.push_back(srv.response.point);
    acM.sendGoal(goal, boost::bind(&Controller::DoneAction<MoveResultConstPtr>, this, _1, _2));
    ROS_WARN("Action MOVE send");

    // acM.start();
  }

  //.start()

}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "controller_node");

	ros::NodeHandle nmh;

  Controller node (&nmh);

	ros::spin();
}
