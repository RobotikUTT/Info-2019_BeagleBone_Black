/** @file move_proc.h
*    @brief define a simple movement action
*    @author Clément de La Bourdonnaye
*/
#ifndef REACH_PROC_H
#define REACH_PROC_H

#include "action_manager/ActionPerformer.hpp"

#include "can_msgs/STMStatus.h"

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
  ros::Timer timerTimeout;
  
  ActionPoint computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, Point robot_pos) override;
  void start() override;
  void cancel() override;

  void movementDone(const can_msgs::Finish::ConstPtr&);
  void timeoutCallback(const ros::TimerEvent&);

};

/**
 *  @}
 */
#endif
