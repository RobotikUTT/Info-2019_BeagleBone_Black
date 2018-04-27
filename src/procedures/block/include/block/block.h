#ifndef BLOCK_PROC_H
#define BLOCK_PROC_H

#include "ros/ros.h"
// #include "ros_can/can"
#include "procedures_msgs/XXX.h"
#include "can_msgs/Point.h"
#include "can_msgs/Finish.h"
#include <vector>
#include <robot_watcher/Services/RobotServices.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<procedures_msgs::XXX> BlockSrv;


class Block
{
public:
  Block(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber sub;

  // ros::Publisher ;
  // ros::Publisher ;
  // ros::Publisher ;
  // ros::Publisher ;

  BlockSrv act;

  std::vector<YYY> fifo;

  void goalCB();
  void preemptCB();
  void analysisCB(const can_msgs::Finish::ConstPtr&);
  inline void sendMsg();


  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
