
#include "reach/reach_proc.hpp"


/**
 * @brief object constructor
 *
 * @param[in] name name of the action server
 */
ReachActionPerfomer::ReachActionPerfomer(std::string name) : ActionPerformer(name) {
  finish_sub        = nh.subscribe("/ALL/Finish",                    1, &ReachActionPerfomer::analysisCB,      this);
  robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &ReachActionPerfomer::GetRobotStatus,  this);

  this->STMGoToAngle_pub        = nh.advertise<can_msgs::Point>("/STM/GoToAngle",         1);
  this->STMGoTo_pub             = nh.advertise<can_msgs::Point>("/STM/GoTo",              1);
  this->STMRotation_pub         = nh.advertise<can_msgs::Point>("/STM/Rotation",          1);
  this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo",  1);
  this->STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement",  1);

  service_ready("procedure", name, 1);
}

/**
 * @brief run action toward a new goal
 */
void ReachActionPerfomer::start() {
  ROS_WARN("Reach: new goal");

  // TODO reach position and angle provided
}


ActionPoint* ReachActionPerfomer::computeActionPoint(std::vector<ai_msgs::Argument> actionArgs, procedures_msgs::OrPoint robot_pos) {
  // TODO
  return new ActionPoint();
}

/**
 * @brief      Callback called when the STM finished all move order
 *
 * @param[in]  msg   The finish message
 */
void ReachActionPerfomer::analysisCB(const can_msgs::Finish::ConstPtr& msg){
  // ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

  if (!actionServer->isActive() || msg->val != 0)
    return;

  TimerTimeout.stop();

  if (!fifo.empty()) {
    sendMsg();
  } else {
    actionPerformed();
  }
}

/**
 * @brief      Sends all Move order in the fifo to the STM.
 */
inline void ReachActionPerfomer::sendMsg() {
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
void ReachActionPerfomer::TimeoutCallback(const ros::TimerEvent& timer){
//reset all goal STM
  // ROS_WARN_STREAM("TIMEOUT");
  can_msgs::Status msg;

  msg.value = RESET_ORDERS;
  STM_AsserManagement_pub.publish(msg);

  if (!fifo.empty()) {
    sendMsg();
  } else {
    actionPerformed();
  }

}

/**
 * @brief      Gets the robot status.
 *
 * @param[in]  msg   The message
 */
[[deprecated("To implement within the action performer")]]
void ReachActionPerfomer::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_status == ROBOT_HALT){
    fifo.clear();
    actionServer->shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "reach_procedure");

  ReachActionPerfomer performer(ros::this_node::getName());
  ros::spin();

  return 0;
}
