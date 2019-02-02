/** @file move_proc.h
*    @brief define a simple movement action
*    @author Clément de La Bourdonnaye
*/
#ifndef REACH_PROC_H
#define REACH_PROC_H

#include "action_manager/ActionPerformer.hpp"

#include "ros/ros.h"

#include "robot_interface/protocol.h"


/**
 * @brief class for the action of reaching a position
 */
class ReachActionPerfomer : public ActionPerformer
{
public:
  ReachActionPerfomer(std::string name);

private:
  ros::Subscriber finish_sub;

  ros::Publisher STMGoToAngle_pub;
  ros::Publisher STMGoTo_pub;
  ros::Publisher STMRotation_pub;
  ros::Publisher STM_AsserManagement_pub;
  ros::Timer TimerTimeout;

  ActionPoint* computeActionPoint(std::vector<ai_msgs::Argument> actionArgs, procedures_msgs::OrPoint robot_pos);
  void start();

  void analysisCB(const can_msgs::Finish::ConstPtr&);
  void TimeoutCallback(const ros::TimerEvent&);

};

/**
 *  @}
 */
#endif
