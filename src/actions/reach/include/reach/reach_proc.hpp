/** @file move_proc.h
*    @brief define a simple movement action
*    @author Clément de La Bourdonnaye
*/
#ifndef REACH_PROC_H
#define REACH_PROC_H

#include "action_manager/ActionPerformer.hpp"

#include "ai_msgs/GetSidedPoint.h"
#include "ai_msgs/Side.h"

#include "interface_msgs/StmMode.h"
#include "interface_msgs/Directions.h"
#include "interface_msgs/CanData.h"

#include "pathfinder/FindPath.h"

#include "geometry_msgs/Pose2D.h"

#include <math.h>

using interface_msgs::Directions;

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
  ros::ServiceClient get_sided_point_srv;
  ros::Publisher can_data_pub;
  
  ros::Timer timerTimeout;
  ros::Timer timerProximity;
  
  geometry_msgs::Pose2D robotPos;

  ActionPoint computeActionPoint(Argumentable* actionArgs, Pose2D robotPos) override;
  void start() override;
  void cancel() override;

  void onCanData(const interface_msgs::CanData::ConstPtr&);
  void timeoutCallback(const ros::TimerEvent&);
  void moveTo(geometry_msgs::Pose2D location, std::string, int);

  void onWaitingResult(bool) override;

  unsigned long convertAngle(long degree) const;

  void processSonars(const Argumentable& data);
	void onProximityTimeout(const ros::TimerEvent& timer);

  bool usePathfinder;

  // Current action blocked
  bool blocked;

  bool proximityStop, forwardStop, backwardStop;

  double proximityTimeout;
  int backTriggerDistance, frontTriggerDistance, escapeDistance;

  int8_t direction;
};

/**
 *  @}
 */
#endif
