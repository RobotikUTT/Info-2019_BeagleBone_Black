/** @file ball_proc.cpp
*    @brief The ball action class
*    
*    @author Alexis CARE
*/
#include "ball/ball_proc.h"


/**
 * @brief      Constructs the object.
 *
 * @param[in]  name  The name of the action server
 */
Ball::Ball(std::string name):
  act(name,false),
  acM("/procedures/move_action", true),
  acC("/procedures/canon_action", true)
  {
    act.registerGoalCallback(   boost::bind(&Ball::goalCB,    this));
    act.registerPreemptCallback(boost::bind(&Ball::preemptCB, this));
    act.start();

    side_sub = nh.subscribe("/ai/side", 1, &Ball::setSide, this );

    service_ready("procedure", "ball", 1 );

  }

/**
 * @brief      The new Goal callback
 */
void Ball::goalCB()
{
  ROS_WARN("Ball: new goal");

  phase = 0;
  point = 0;
  procedures_msgs::BallGoal::ConstPtr msg = act.acceptNewGoal();

  objectif = GroupBalls(msg->tube_pose.x, msg->tube_pose.y, msg->tube_pose.rot,
                        msg->param[0] , msg->param[1],
                        msg->shoot_pose.x, msg->shoot_pose.y, msg->shoot_pose.rot,
                        side);

  sendMsg();
}

/**
 * @brief      The preempt Callback
 */
void Ball::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

/**
 * @brief      Sends all Ball order in the correspondate phase to the Arduino.
 */
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
      //error
      ROS_WARN_STREAM("default");
    }
  }
  // if(phase%2 == 0){
  //   ROS_INFO("MOVE");
  // } else {
  //   ROS_INFO("PLIERS");
  // }
}

/**
 * @brief      Sets the Robot side.
 *
 * @param[in]  msg   The SetSide message
 */
void Ball::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

/**
 * @brief      The action done callback
 * 
 * @param[in]  state    The state of the action
 * @param[in]  result   The result message
 *
 * @tparam     doneMsg  The action result type
 */
template <class doneMsg>
void Ball::DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result){

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
    point += result->points_done;

    phase ++;

    sendMsg();
  }
}

/**
 * @brief      Gets the robot status.
 *
 * @param[in]  msg   The RobotStatus message
 */
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
