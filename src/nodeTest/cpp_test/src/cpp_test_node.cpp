#include "ros/ros.h"


void init(ros::NodeHandle nmh) {

  std::map<std::string,int> ma;
  int s;
  if(nmh.getParam("/ai/controller/robot_pos", ma))
  {
    ROS_INFO("Got param: %d", ma["x"]);
    ROS_INFO("Got param: %d", ma["y"]);
    // ROS_INFO("Got param");
  }
  else
  {
    ROS_ERROR("Failed to get param 'my_param'");
   }
}



int main(int argc, char *argv[]) {
  ros::init(argc,argv, "testcpp");
  ros::NodeHandle nmh;

  init(nmh);

	// ros::NodeHandle nmh;
  //
  // Controller node (&nmh);

	ros::spin();
}
