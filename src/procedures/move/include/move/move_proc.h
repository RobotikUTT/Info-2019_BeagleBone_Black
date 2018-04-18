#ifndef MOVE_PROC_H
#define MOVE_PROC_H

#include "ros/ros.h"

typedef actionlib::SimpleActionServer<procedures_msgs::XX> MoveAction;


class Move
{
public:
  Move(std::string name);

private:

  ros::NodeHandle nh;

  ros::Subscriber sub;

  MoveAction act;

  // procedures_msgs::XXFeedback feedback
  // procedures_msgs::XXResult result
};


#endif
