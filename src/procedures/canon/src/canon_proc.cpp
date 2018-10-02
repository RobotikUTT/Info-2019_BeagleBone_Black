/** @file canon_proc.h
*    @brief canon action server class
*/
#include "canon/canon_proc.h"


/**
 * @brief      Constructs the object.
 *
 * @param[in]  name  The name of the action server
 */
Canon::Canon(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(boost::bind(&Canon::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Canon::preemptCB, this));
    act.start();

    finish_sub = nh.subscribe("/ALL/Finish", 1, &Canon::analysisCB, this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Canon::GetRobotStatus, this);

    ARDUINO_canon_pub = nh.advertise<can_msgs::ThrowBalls>("/ARDUINO/ThrowBalls", 10);

    service_ready("procedure", "canon", 1 );
  }

/**
 * @brief      The new Goal callback
 */
void Canon::goalCB()
{
  // ROS_WARN("Canon: new goal");

  bool temp = !act.isActive();
  procedures_msgs::CanonGoal::ConstPtr msg = act.acceptNewGoal();
  sendMsg();
}

/**
 * @brief      The preempt Callback
 */
void Canon::preemptCB()
{
  ROS_DEBUG("Canon; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

/**
 * @brief      Callback called when the Arduino finished all canon order
 *
 * @param[in]  msg   The Finish message
 */
void Canon::analysisCB(const can_msgs::Finish::ConstPtr& msg)
{
  // ROS_WARN_STREAM("Canon; FINISH : state "<< act.isActive());
  if (!act.isActive() || msg->val != CANON)
    return;

    procedures_msgs::CanonResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
}

/**
 * @brief      Sends a Canon message.
 */
inline void Canon::sendMsg() {
  can_msgs::ThrowBalls msg;

  ARDUINO_canon_pub.publish(msg);

}

/**
 * @brief      Gets the robot status.
 *
 * @param[in]  msg   The RobotStatus message
 */
void Canon::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "canon_procedure");

  Canon pliering(ros::this_node::getName());
  ros::spin();

  return 0;
}
