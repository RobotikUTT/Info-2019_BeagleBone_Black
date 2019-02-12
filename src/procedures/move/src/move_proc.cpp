/** @file move_proc.cpp
*    @brief The Move action class
*    
*    @author Alexis CARE
*/
#include "move/move_proc.h"


/**
 * @brief      Constructs the object.
 *
 * @param[in]  name  The name of the  action server
 */
Move::Move(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(   boost::bind(&Move::goalCB,    this));
    act.registerPreemptCallback(boost::bind(&Move::preemptCB, this));
    act.start();

    finish_sub        = nh.subscribe("/ALL/Finish",                    1, &Move::analysisCB,      this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Move::GetRobotStatus,  this);

    this->STMGoToAngle_pub        = nh.advertise<can_msgs::Point>("/STM/GoToAngle",         1);
    this->STMGoTo_pub             = nh.advertise<can_msgs::Point>("/STM/GoTo",              1);
    this->STMRotation_pub         = nh.advertise<can_msgs::Point>("/STM/Rotation",          1);
    this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo",  1);
    this->STM_AsserManagement_pub = nh.advertise<can_msgs::STMStatus>("/STM/AsserManagement",  1);

    service_ready("procedure", "move", 1 );
  }

/**
 * @brief      The new Goal callback
 */
void Move::goalCB(){
  // ROS_WARN("Move: new goal");

  bool temp = !act.isActive();
  procedures_msgs::MoveGoal::ConstPtr msg = act.acceptNewGoal();
  for (int i = 0; i < msg->points.size(); i++) {
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].Opoint.x << "; y: " << msg->points[i].Opoint.y <<"; angle: "<< msg->points[i].Opoint.rot<< "; type: "<< (int)msg->points[i].type << "}" );
    fifo.push_back(MovePoint(msg->points[i].Opoint.x, msg->points[i].Opoint.y, msg->points[i].Opoint.rot, msg->points[i].type, msg->points[i].direction, msg->points[i].timeout));
  }
  if(temp){
    sendMsg();
  }
}
/**
 * @brief      The preempt Callback
 */
void Move::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

/**
 * @brief      Callback called when the STM finished all move order
 *
 * @param[in]  msg   The finish message
 */
void Move::analysisCB(const can_msgs::Finish::ConstPtr& msg){
  // ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

  if (!act.isActive() || msg->val != 0)
    return;

  TimerTimeout.stop();

  if (!fifo.empty()) {
    sendMsg();
  } else {
    procedures_msgs::MoveResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  }
}

/**
 * @brief      Sends all Move order in the fifo to the STM.
 */
inline void Move::sendMsg() {
  can_msgs::Point msg;
  //direction
  // ROS_INFO_STREAM("size: " << fifo.size());
  // ROS_INFO_STREAM("Point send: { x: " << fifo.front()._x << "; y: " << fifo.front()._y <<"; angle: "<< fifo.front()._angle<< " timeout: "<< fifo.front()._timeout << "}" );
  if (fifo.front()._timeout != 0) {
    switch (fifo.front()._move_type) {
      case GO_TO_ANGLE:
      msg.pos_x = fifo.front()._x;
      msg.pos_y = fifo.front()._y;
      msg.angle = fifo.front()._angle;
      msg.direction = fifo.front()._direction;
      STMGoToAngle_pub.publish(msg);
      break;
      case GO_TO:
      msg.pos_x = fifo.front()._x;
      msg.pos_y = fifo.front()._y;
      msg.direction = fifo.front()._direction;
      STMGoTo_pub.publish(msg);
      break;
      case ROTATION:
      msg.angle = fifo.front()._angle;
      STMRotation_pub.publish(msg);
      break;
      case ROTATION_NO_MODULO:
      msg.angle = fifo.front()._angle;
      STMRotationNoModulo_pub.publish(msg);
      break;
    }
    // ROS_INFO_STREAM("TIMEOUT SET");
    TimerTimeout = nh.createTimer(ros::Duration(fifo.front()._timeout), &Move::TimeoutCallback , this, true);
    fifo.erase(fifo.begin());

  } else {
    while (!fifo.empty() && fifo.front()._timeout == 0) {

      switch (fifo.front()._move_type) {
        case GO_TO_ANGLE:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
        msg.angle = fifo.front()._angle;
        msg.direction = fifo.front()._direction;
        STMGoToAngle_pub.publish(msg);
        break;
        case GO_TO:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
        msg.direction = fifo.front()._direction;
        STMGoTo_pub.publish(msg);
        break;
        case ROTATION:
        msg.angle = fifo.front()._angle;
        STMRotation_pub.publish(msg);
        break;
        case ROTATION_NO_MODULO:
        msg.angle = fifo.front()._angle;
        STMRotationNoModulo_pub.publish(msg);
        break;
      }
      fifo.erase(fifo.begin());
      ros::Duration(0.1).sleep();
    }
  }
}

/**
 * @brief      Send new order if the robot is too slow
 * 
 * @details    We concider that the robot is blocked at the end of the timer 
 *
 * @param[in]  timer  The timer event
 */
void Move::TimeoutCallback(const ros::TimerEvent& timer){
//reset all goal STM
  // ROS_WARN_STREAM("TIMEOUT");
  can_msgs::STMStatus msg;

  msg.value = can_msgs::STMStatus::RESET_ORDERS;
  STM_AsserManagement_pub.publish(msg);

  if (!fifo.empty()) {
    sendMsg();
  } else {
    procedures_msgs::MoveResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  }

}

/**
 * @brief      Gets the robot status.
 *
 * @param[in]  msg   The message
 */
void Move::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_status == RobotStatus::ROBOT_HALT){
    fifo.clear();
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_procedure");

  Move moving(ros::this_node::getName());
  ros::spin();

  return 0;
}
