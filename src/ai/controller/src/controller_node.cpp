#include "controller/controller_node.h"

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const> MoveResultConstPtr;
typedef boost::shared_ptr< ::procedures_msgs::BlockResult const> BlockResultConstPtr;

Controller::Controller(ros::NodeHandle* n):
acM("/procedures/move_action", true),
acB("/procedures/block_action", true){
  nh = *n;

  status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Controller::GetRobotStatus, this);
  //sub for sonar data
  //sonar_sub = nh.subscribe("/Arduino/sonars", 1, &Controller::GetSonars, this);
  robot_pos_sub = nh.subscribe("/STM/Position", 1, &Controller::GetRobotPose, this);
  robot_speed_sub = nh.subscribe("/STM/GetSpeed", 1, &Controller::GetRobotSpeed, this);
  side_sub = nh.subscribe("side", 1, &Controller::setSide, this);
  sonar_distance_sub = nh.subscribe("/ARDUINO/SonarDistance", 1, &Controller::processSonars, this);


  emergency_stop_pub = nh.advertise<ai_msgs::EmergencyStop>("emergency", 1);
  STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
  STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);

  side = SIDE_GREEN;

  clientD = nh.serviceClient<ai_msgs::CurrentActionDone>("scheduler/currentActionDone");
  clientA = nh.serviceClient<ai_msgs::GetActionToDo>("scheduler/actionToDo");
  clientA.waitForExistence();

  acM.waitForServer();
  acB.waitForServer();
  service_ready("ai", "controller", 1 );

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
 ROS_INFO_STREAM("robot pos_x: " << msg->pos_x);
 ROS_INFO_STREAM("robot pos_y: " << msg->pos_y);
 ROS_INFO_STREAM("robot angle: " << msg->angle);
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
    int x,y,angle;
    nh.getParam("controller/robot_pos/x", x);
    nh.getParam("controller/robot_pos/y", y);
    nh.getParam("controller/robot_pos/angle", angle);

      if(side){
        x = (1500 - x) + 1500;
        angle = -angle;
      }
      //init pos STM
      can_msgs::Point msg;

      robot_pos_x = x;
      robot_pos_y = y;
      robot_angle = angle;

      msg.pos_x = x;
      msg.pos_y = y;
      msg.angle = angle;

      STM_SetPose_pub.publish(msg);
      can_msgs::Status msg2;
      msg2.value = START;
      STM_AsserManagement_pub.publish(msg2);
      SetAction();
      //set action to true
  }
}

void Controller::GetRobotSpeed(const can_msgs::CurrSpeed::ConstPtr& msg)
{
  ROS_WARN("GET SPEED");
  int16_t linearSpeed = msg->linear_speed;
  

  if( linearSpeed > 0 )
  {
    direction = FORWARD;
    ROS_INFO("FORWARD");
  }
  else if (linearSpeed < 0)
  {
    direction = BACKWARD;
    ROS_INFO("BACKWARD");
  }
  else
  {
    direction = NONE;
    ROS_INFO("NONE");
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

  } else if (action_val == BLOCK){

    procedures_msgs::BlockGoal goal;

    goal.block_action =srv.response.block_action;
    goal.depot = srv.response.depot;

    acB.sendGoal(goal, boost::bind(&Controller::DoneAction<BlockResultConstPtr>, this, _1, _2));

  }
}

void Controller::processSonars(const can_msgs::SonarDistance::ConstPtr& msg)
{
  bool last_emergency_value = emergency_stop;
  ROS_WARN("PROCESS SONAR");
  uint8_t front_left,front_right,
          left, right, back;
  
  front_left = msg->dist_front_left;
  front_right = msg->dist_front_right;
  left = msg->dist_left;
  right = msg->dist_right;
  back = msg->dist_back;
  
  emergency_stop = false;
  if ( direction == FORWARD)
  {
    if (front_left <= SONAR_MIN_DIST ||
        front_right <= SONAR_MIN_DIST)
    {
      emergency_stop = true;
      ROS_INFO("EMG FORWARD");
    }
  }
  else if ( direction == BACKWARD)
  {
    if ( back <= SONAR_MIN_DIST)
    {
      emergency_stop = true;
      ROS_INFO("EMG BACKWARD");
    }
  }
  else
  {

    
    ROS_INFO("NO EMG");
  }

  if (last_emergency_value != emergency_stop)
  {
    ai_msgs::EmergencyStop emergency_msg;
    emergency_msg.emergency_set = emergency_stop;
    emergency_stop_pub.publish(emergency_msg);

    can_msgs::Status can_msg;
    if (emergency_stop)
    {
      ROS_INFO("SEND EMG MSG");
      can_msg.value = SETEMERGENCYSTOP;
    }
    else
    {
      ROS_INFO("SEND NO EMG MSG");
      can_msg.value = UNSETEMERGENCYSTOP;
    }
    STM_AsserManagement_pub.publish(can_msg);

  }



  
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "controller_node");

	ros::NodeHandle nmh;

  Controller node (&nmh);

	ros::spin();
}
