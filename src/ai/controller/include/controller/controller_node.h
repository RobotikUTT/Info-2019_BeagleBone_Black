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
#include "ai_msgs/EmergencyStop.h"
#include "can_msgs/Point.h"
#include "can_msgs/Status.h"
#include "can_msgs/SonarDistance.h"
#include "can_msgs/Frame.h"
#include "can_msgs/CurrSpeed.h"
#include "procedures_msgs/MoveAction.h"
#include "procedures_msgs/BlockAction.h"
#include "action/action_define.h"
#include <actionlib/client/simple_action_client.h>
#include "robot_interface/protocol.h"
//include action

#define FOREWARD        1
#define BACKWARD        -1
#define NONE            0

#define SONAR_MIN_DIST  0

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
  ros::Subscriber robot_speed_sub;
  ros::Subscriber sonar_distance_sub;
  ros::Subscriber side_sub;

  ros::Publisher emergency_stop_pub;
  ros::Publisher STM_SetPose_pub;
  ros::Publisher STM_AsserManagement_pub;
  ros::NodeHandle nh;

  //robot pos
  int robot_pos_x;
  int robot_pos_y;
  int robot_angle;

  uint8_t robot_status;
  int8_t direction;
  bool emergency_stop;
  bool side;

  int8_t action_val;

  int points_done;

  ClientMove acM;
  ClientBlock acB;
  // std::map<std::string, value> map;

  void GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg);
  void GetRobotPose(const can_msgs::Point::ConstPtr& msg);
  void GetRobotSpeed(const can_msgs::CurrSpeed::ConstPtr& msg);
  void SetAction();
  void setSide(const ai_msgs::SetSide::ConstPtr& msg);
  void processSonars(const can_msgs::SonarDistance::ConstPtr& msg);

  template <class doneMsg>
  void DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result);

  // void GetSonars(const can_msgs::Sonars::ConstPtr & msg);

};
#endif
