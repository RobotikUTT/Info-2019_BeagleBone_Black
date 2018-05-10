#include "controller/controller_node.h"

typedef boost::shared_ptr< ::procedures_msgs::MoveResult const>   MoveResultConstPtr;
typedef boost::shared_ptr< ::procedures_msgs::BlockResult const>  BlockResultConstPtr;
typedef boost::shared_ptr< ::procedures_msgs::BallResult const>   BallResultConstPtr;

Controller::Controller(ros::NodeHandle* n):
acM("/procedures/move_action", true),
acBl("/procedures/ball_action", true),
acB("/procedures/block_action", true){
  nh = *n;

  status_sub          = nh.subscribe("robot_watcher/robot_status",  1, &Controller::GetRobotStatus,     this);
  robot_pos_sub       = nh.subscribe("/STM/Position",               1, &Controller::GetRobotPose,       this);
  robot_speed_sub     = nh.subscribe("/STM/GetSpeed",               1, &Controller::GetRobotSpeed,      this);
  side_sub            = nh.subscribe("side",                        1, &Controller::setSide,            this);
  nodes_status_sub    = nh.subscribe("robot_watcher/nodes_status",  1, &Controller::checkForPANEL,      this);
  sonar_distance_sub  = nh.subscribe("/ARDUINO/SonarDistance",      1, &Controller::processSonars,      this);
  robot_blocked_sub   = nh.subscribe("/STM/RobotBlocked",           1, &Controller::processRobotBlocked,this);

  emergency_stop_pub      = nh.advertise<ai_msgs::EmergencyStop>("emergency",             1);
  STM_SetPose_pub         = nh.advertise<can_msgs::Point>       ("/STM/SetPose",          1);
  STM_AsserManagement_pub = nh.advertise<can_msgs::Status>      ("/STM/AsserManagement",  1);
  PANEL_Point_pub         = nh.advertise<std_msgs::Int8>        ("/PANEL/AddPoint",       1);
  ARDUINO_Move_Pliers_pub         = nh.advertise<std_msgs::Int8>        ("/ARDUINO/MovePliers",       1);

  clientD = nh.serviceClient<ai_msgs::CurrentActionDone>  ("scheduler/currentActionDone");
  clientA = nh.serviceClient<ai_msgs::GetActionToDo>      ("scheduler/actionToDo");
  clientA.waitForExistence();

  side            = SIDE_GREEN;
  direction       = NONE;
  emergency_stop  = false;
  PANLEUp         = 0;
  points_done     = 0;

  acM.waitForServer();
  acB.waitForServer();
  service_ready("ai", "controller", 1 );

}

void Controller::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

//function to get robot pos
void Controller::GetRobotPose(const can_msgs::Point::ConstPtr & msg){
// ROS_INFO_STREAM("robot pos_x: " << msg->pos_x
//  << " robot pos_y: " << msg->pos_y
//  << " robot angle: " << msg->angle);
  robot_pos_x = msg->pos_x;
  robot_pos_y = msg->pos_y;
  robot_angle = msg->angle;
}

void Controller::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  robot_status = msg->robot_watcher;

  if(robot_status == ROBOT_READY){
    //init card
    // ROS_DEBUG("Robot Ready");
    std_msgs::Int8 msg;
    msg.data = 1;

    ARDUINO_Move_Pliers_pub.publish(msg);


  } else if (robot_status == ROBOT_RUNNING){
    // ROS_DEBUG("Robot Running");
    int x,y,angle;
    nh.getParam("controller/robot_pos/x", x);
    nh.getParam("controller/robot_pos/y", y);
    nh.getParam("controller/robot_pos/angle", angle);

      if(side){ //if orange
        y = (1500 - y) + 1500;
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
  } else if (robot_status == ROBOT_HALT){

    acM.cancelAllGoals();
    acB.cancelAllGoals();

    can_msgs::Status msg;
    msg.value = STOP;
    STM_AsserManagement_pub.publish(msg);

    msg.value = RESET_ORDERS;
    STM_AsserManagement_pub.publish(msg);
  }
}

void Controller::checkForPANEL(const ai_msgs::NodesStatus::ConstPtr & msg){
  for (int i = 0; i < msg->nodes_ready.size(); i++){
    if(msg->nodes_ready[i] == "/board/PANEL"){
        PANLEUp = 1;
        nodes_status_sub.shutdown();
        sendPoint();
    }
  }
}

void Controller::sendPoint(){
 // ROS_INFO_STREAM("sendPoint point " << points_done);
  std_msgs::Int8 msg;
  msg.data = points_done;
  PANEL_Point_pub.publish(msg);
  points_done = 0;
}

void Controller::GetRobotSpeed(const can_msgs::CurrSpeed::ConstPtr& msg)
{
  int16_t linearSpeed = msg->linear_speed;
  int16_t leftSpeed = msg->left_speed;
  int16_t rightSpeed = msg->right_speed;

//  ROS_INFO_STREAM("SPEEDS|linear: " << linearSpeed
//  << " left: " << leftSpeed
//  << " right: " << rightSpeed);

  if( linearSpeed > 0 ){
    direction = FORWARD;
  } else if (linearSpeed < 0) {
    direction = BACKWARD;
  } else {
    direction = NONE;
  }
}

template <class doneMsg>
void Controller::DoneAction( const actionlib::SimpleClientGoalState& state,
                        const doneMsg & result){
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  //send point
  points_done += result->points_done;
  if(PANLEUp){
    sendPoint();
  }

  if (robot_status != ROBOT_HALT){
    //set old action to done
    ai_msgs::CurrentActionDone srv;
    srv.request.done = result->done;
    // ROS_INFO_STREAM("Action in state " << result->done);
    if (!clientD.call(srv)) {
      ROS_ERROR("Failed to call service currentActionDone");
    }

    SetAction();
  }

}

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

  if(action_val == MOVE){
    // ROS_WARN("Action MOVE");
    procedures_msgs::MoveGoal goal;
    goal.points.push_back(srv.response.point);
    acM.sendGoal(goal, boost::bind(&Controller::DoneAction<MoveResultConstPtr>, this, _1, _2));
    // ROS_WARN("Action MOVE send");

  } else if (action_val == BLOCK){

    procedures_msgs::BlockGoal goal;

    goal.block_action =srv.response.action_pos;
    goal.depot = srv.response.depot_pos;

    acB.sendGoal(goal, boost::bind(&Controller::DoneAction<BlockResultConstPtr>, this, _1, _2));

  } else if (action_val == BALL){
    procedures_msgs::BallGoal goal;

    goal.tube_pose =srv.response.action_pos;
    goal.shoot_pose = srv.response.depot_pos;
    goal.param = srv.response.param;

    acBl.sendGoal(goal, boost::bind(&Controller::DoneAction<BallResultConstPtr>, this, _1, _2));

  }
}

void Controller::processSonars(const can_msgs::SonarDistance::ConstPtr& msg)
{
  bool last_emergency_value = emergency_stop;
  uint8_t front_left,front_right,
          back_left, back_right;

  front_left = msg->dist_front_left;
  front_right = msg->dist_front_right;
  back_left = msg->dist_back_left;
  back_right = msg->dist_back_right;

  // ROS_INFO_STREAM("DIST|" << front_left << "|" << front_right
  // << "|" << left << "|"  << right << "|" << back);
  ROS_INFO("DIST|%u|%u|%u|%u|",front_left,
    front_right, back_left, back_right);

  emergency_stop = false;
  if ( direction == FORWARD){
    if (front_left <= SONAR_MIN_DIST_FORWARD + 6 ||
        front_right <= SONAR_MIN_DIST_FORWARD + 16 )
    {
      emergency_stop = true;
    }
  } else if ( direction == BACKWARD){
    if ( back_left <= SONAR_MIN_DIST_BACKWARD ||
         back_right <= SONAR_MIN_DIST_BACKWARD  ){
      emergency_stop = true;
    }
  } else {
    if (front_left <= SONAR_MIN_DIST_FORWARD ||
        front_right <= SONAR_MIN_DIST_FORWARD ||
        back_left <= SONAR_MIN_DIST_BACKWARD ||
        back_right <= SONAR_MIN_DIST_BACKWARD)
    {
      emergency_stop = true;
    }
  }

  if (last_emergency_value != emergency_stop){
    ai_msgs::EmergencyStop emergency_msg;
    emergency_msg.emergency_set = emergency_stop;
    emergency_stop_pub.publish(emergency_msg);

    if (emergency_stop){
      ROS_WARN("SET EMG");
    } else {
      ROS_WARN("UNSET EMG");
    }

    can_msgs::Status can_msg;
    if (emergency_stop){
      can_msg.value = SETEMERGENCYSTOP;
    } else {
      can_msg.value = UNSETEMERGENCYSTOP;
    }
    STM_AsserManagement_pub.publish(can_msg);
  }
}

void Controller::processRobotBlocked(const can_msgs::RobotBlocked::ConstPtr& msg)
{
  ROS_WARN("Robot blocked");
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "controller_node");

	ros::NodeHandle nmh;

  Controller node (&nmh);

	ros::spin();
}
