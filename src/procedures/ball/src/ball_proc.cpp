#include "ball/ball_proc.h"


Ball::Ball(std::string name):
  act(name,false),
  acM("/procedures/move_action", true),
  acC("/procedures/canon_action", true)
  {
    act.registerGoalCallback(boost::bind(&Ball::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Ball::preemptCB, this));
    act.start();

    // finish_sub = nh.subscribe("/ALL/Finish", 10, &Ball::analysisCB, this);
    side_sub = nh.subscribe("/ai/side", 1, &Ball::setSide, this );

    // ARDUINO_pliers_pub = nh.advertise<can_msgs::ActionPliers>("/ARDUINO/ActionPliers", 10);

    // this->STMGoToAngle_pub = nh.advertise<can_msgs::Point>("/STM/GoToAngle", 1);
    // this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
    // this->STMRotation_pub = nh.advertise<can_msgs::Point>("/STM/Rotation", 1);
    // this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo", 1);


    //subs
    // sub = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    service_ready("procedure", "ball", 1 );

  }

void Ball::goalCB()
{
  ROS_WARN("Ball: new goal");

  // bool temp = !act.isActive();
  phase = 0;
  point = 0;
  procedures_msgs::BallGoal::ConstPtr msg = act.acceptNewGoal();
  // for (int i = 0; i < msg->points.size(); i++) {
  //   fifo.push_back();
  // }

  objectif = GroupBalls(msg->tube_pose.x, msg->tube_pose.y, msg->tube_pose.rot,
                        msg->param[0] , msg->param[1],
                        msg->shoot_pose.x, msg->shoot_pose.y, msg->shoot_pose.rot,
                        side);

  sendMsg();

  // if(temp){
  //   sendMsg();
  // }
  // ROS_WARN("Ball: end");

}

void Ball::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

// void Ball::analysisCB(const can_msgs::Finish::ConstPtr& msg)
// {
//   // make sure that the action hasn't been canceled
//   // ROS_WARN_STREAM("Ball; FINISH : state "<< act.isActive());
//
//   if (!act.isActive() || msg->val != BLOCK)
//     return;
//
//   phase++;
//
//   // if (!fifo.empty()) {
//   //   /* code */
//   //   ROS_INFO_STREAM("Ball; FiFo : not empty");
//   sendMsg();
//
//
//   // } else {
//   //   procedures_msgs::MoveResult result_;
//   //   result_.done = 1;
//   //   act.setSucceeded(result_);
//   // }
// }

void Ball::sendMsg() {

  if (!act.isActive())
      return;

  ROS_WARN_STREAM("phase: " << (int)phase);


  switch (phase) {
    case 0:{
      // move
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[1].x;
      temp.Opoint.y = objectif.proc_point[1].y;
      temp.Opoint.rot = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      acM.sendGoal(goal, boost::bind(&Ball::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 1:{
      // wait
      ROS_WARN("WAIT");
      ros::Duration(5).sleep();
      phase ++;
      ROS_WARN("WAIT END");
    }
    case 2:{
      // move
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[2].x;
      temp.Opoint.y = objectif.proc_point[2].y;
      temp.Opoint.rot = objectif.proc_point[2].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      acM.sendGoal(goal, boost::bind(&Ball::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 3:{
      // shoot
      procedures_msgs::CanonGoal goal;

      acC.sendGoal(goal, boost::bind(&Ball::DoneAction<CanonResultConstPtr>, this, _1, _2));
      break;
    }
    case 4:{
      procedures_msgs::BallResult result_;
      result_.done = 1;
      result_.points_done = 10;
      act.setSucceeded(result_);
      break;
    }

    default: {
      ROS_WARN_STREAM("default");
      //error
    }
  }
  // if(phase%2 == 0){
  //   ROS_INFO("MOVE");
  // } else {
  //   ROS_INFO("PLIERS");
  // }

  // fifo.erase(fifo.begin());
}

void Ball::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

template <class doneMsg>
void Ball::DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result){

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
    point += result->points_done;

    phase ++;

    sendMsg();
  }
}

void Ball::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_procedure");

  Ball bllAction(ros::this_node::getName());
  ros::spin();

  return 0;
}
