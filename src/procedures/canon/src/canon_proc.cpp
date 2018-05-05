#include "canon/canon_proc.h"


Canon::Canon(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(boost::bind(&Canon::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Canon::preemptCB, this));
    act.start();

    finish_sub = nh.subscribe("/ALL/Finish", 1, &Canon::analysisCB, this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Canon::GetRobotStatus, this);

    ARDUINO_canon_pub = nh.advertise<can_msgs::ThrowBalls>("/ARDUINO/ThrowBalls", 10);

    //subs
    service_ready("procedure", "canon", 1 );

  }

void Canon::goalCB()
{
  // ROS_WARN("Canon: new goal");

  bool temp = !act.isActive();
  procedures_msgs::CanonGoal::ConstPtr msg = act.acceptNewGoal();
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].end_x << "; y: " << msg->points[i].end_y <<"; angle: "<< msg->points[i].end_angle<< "; type: "<< (int)msg->points[i].type << "}" );
  // fifo.push_back(CanonCommand(msg->action, msg->level));
  //
  // if(temp){
    sendMsg();
  // }
}
void Canon::preemptCB()
{
  ROS_DEBUG("Canon; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

void Canon::analysisCB(const can_msgs::Finish::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  // ROS_WARN_STREAM("Canon; FINISH : state "<< act.isActive());

  if (!act.isActive() || msg->val != CANON)
    return;

  // if (!fifo.empty()) {
  //   // ROS_INFO_STREAM("Canon; FiFo : not empty");
  //   sendMsg();
  //
  // } else {
    procedures_msgs::CanonResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  // }
}

inline void Canon::sendMsg() {
  can_msgs::ThrowBalls msg;

  ARDUINO_canon_pub.publish(msg);

}

void Canon::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "canon_procedure");

  Canon pliering(ros::this_node::getName());
  ros::spin();

  return 0;
}

/*
callback:
  check if other point
    send new point
  no point
    succeeded

can send multiple point

need to wait STM msg to continue

*/
