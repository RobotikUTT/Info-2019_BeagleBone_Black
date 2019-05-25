/** @file move_proc.h
*    @brief define a simple movement action
*    @author Clément de La Bourdonnaye
*/
#ifndef REACH_PROC_H
#define REACH_PROC_H

#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/GetMapSize.h"
#include "ai_msgs/Side.h"

#include "interface_msgs/StmMode.h"
#include "interface_msgs/Directions.h"
#include "interface_msgs/CanData.h"

#include "pathfinder/FindPath.h"

#include "geometry_msgs/Pose2D.h"

#include <math.h>

/**
 * @brief class for the action of reaching a position
 */
class ReachActionPerformer : public ActionPerformer
{
public:
  ReachActionPerformer(std::string name);

private:
  ros::Subscriber can_data_sub;
  ros::ServiceClient pathfinder_srv;
  ros::ServiceClient get_map_data_srv;
  ros::Publisher can_data_pub;
  
  ros::Timer timerTimeout;
  
  geometry_msgs::Pose2D robotPos;

  ActionPoint computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) override;
  void start() override;
  void cancel() override;

  void onCanData(const interface_msgs::CanData::ConstPtr&);
  void timeoutCallback(const ros::TimerEvent&);
  void moveTo(geometry_msgs::Pose2D location);

  void onWaitingResult(bool) override;

  bool usePathfinder;
};

/**
 *  @}
 */
#endif
