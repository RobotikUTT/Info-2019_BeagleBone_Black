/** @file move_proc.h
*    @brief define a idle action
*    @author Clément de La Bourdonnaye
*/
#ifndef IDLE_PROC_H
#define IDLE_PROC_H

#include "action_manager/ActionPerformer.hpp"

/**
 * @brief class for the action of being
 */
class IdleActionPerfomer : public ActionPerformer
{
public:
  IdleActionPerfomer(std::string name);

private:
  ros::Subscriber finish_sub;

  ActionPoint computeActionPoint(Argumentable* actionArgs, OrientedPoint robot_pos) override;
  void start() override;
  void cancel() override;

};

/**
 *  @}
 */
#endif
