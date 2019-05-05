/** @file move_proc.h
*    @brief define a simple movement action
*    @author Clément de La Bourdonnaye
*/
#ifndef REACH_PROC_H
#define REACH_PROC_H

#include "action_manager/ActionPerformer.hpp"

#include "interface_msgs/StmMode.h"
#include "interface_msgs/Directions.h"
#include "interface_msgs/CanData.h"

/**
 * @brief class for the action of reaching a position
 */
class ReachActionPerfomer : public ActionPerformer
{
public:
  ReachActionPerfomer(std::string name);

private:
  ros::Subscriber can_data_sub;

  ros::Publisher can_data_pub;
  ros::Timer timerTimeout;
  
  ActionPoint computeActionPoint(Argumentable* actionArgs, Pose2D robot_pos) override;
  void start() override;
  void cancel() override;

  void onCanData(const interface_msgs::CanData::ConstPtr&);
  void timeoutCallback(const ros::TimerEvent&);

};

/**
 *  @}
 */
#endif
