#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include "robot_watcher/Services/RobotServices.h"

#include "robot_watcher/RStatus/State.h"
#include "robot_watcher/RStatus/Side.h"

#include "ai_msgs/RobotStatus.h"
#include "ai_msgs/SetSide.h"
#include "ai_msgs/GetActionToDo.h"
#include "ai_msgs/CurrentActionDone.h"
#include "ai_msgs/NodesStatus.h"
#include "std_msgs/Int8.h"
#include "can_msgs/Point.h"
#include "can_msgs/Status.h"
#include "procedures_msgs/MoveAction.h"
#include "procedures_msgs/BlockAction.h"
#include "action/action_define.h"
#include <actionlib/client/simple_action_client.h>
//include action

typedef actionlib::SimpleActionClient<procedures_msgs::MoveAction> ClientMove;
typedef actionlib::SimpleActionClient<procedures_msgs::BlockAction> ClientBlock;

class Controller
{
public:
  Controller(ros::NodeHandle* n);


private:
  ros::ServiceClient clientA;
  ros::ServiceClient clientD;

  ros::Subscriber status_sub;
  ros::Subscriber robot_pos_sub;
  ros::Subscriber nodes_status_sub;
  // ros::Subscriber sonar_sub;
  ros::Subscriber side_sub;

  // ros::Publisher emergency_stop_pub;
  ros::Publisher STM_SetPose_pub;
  ros::Publisher STM_AsserManagement_pub;
  ros::Publisher PANEL_Point_pub;
  ros::NodeHandle nh;

  //robot pos
  int robot_pos_x;
  int robot_pos_y;
  int robot_angle;

  uint8_t robot_status;
  // bool emergency_stop;
  bool side;
  bool PANLEUp;

  int8_t action_val;

  int points_done;

  ClientMove acM;
  ClientBlock acB;
  // std::map<std::string, value> map;

  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
  void GetRobotPose(const can_msgs::Point::ConstPtr& msg);
  void SetAction();
  void setSide(const ai_msgs::SetSide::ConstPtr& msg);
  void checkForPANEL(const ai_msgs::NodesStatus::ConstPtr & msg);
  void sendPoint();

  template <class doneMsg>
  void DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result);

  // void GetSonars(const can_msgs::Sonars::ConstPtr & msg);

};
#endif
