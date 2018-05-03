#include "pliers/pliers_proc.h"


Pliers::Pliers(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(boost::bind(&Pliers::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Pliers::preemptCB, this));
    act.start();

    finish_sub = nh.subscribe("/ALL/Finish", 1, &Pliers::analysisCB, this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Pliers::GetRobotStatus, this);

    ARDUINO_pliers_pub = nh.advertise<can_msgs::ActionPliers>("/ARDUINO/ActionPliers", 10);
    // this->STMGoToAngle_pub = nh.advertise<can_msgs::Point>("/STM/GoToAngle", 1);
    // this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
    // this->STMRotation_pub = nh.advertise<can_msgs::Point>("/STM/Rotation", 1);
    // this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo", 1);


    //subs
    // sub = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    service_ready("procedure", "pliers", 1 );

  }

void Pliers::goalCB()
{
  // ROS_WARN("Pliers: new goal");

  bool temp = !act.isActive();
  // procedures_msgs::PliersGoal::ConstPtr msg = act.acceptNewGoal();
  for (int i = 0; i < msg->points.size(); i++) {
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].end_x << "; y: " << msg->points[i].end_y <<"; angle: "<< msg->points[i].end_angle<< "; type: "<< (int)msg->points[i].type << "}" );
    fifo.push_back(PliersCommand(msg->action, msg->level));
  }
  //
  if(temp){
    sendMsg();
  }
}
void Pliers::preemptCB()
{
  ROS_DEBUG("Pliers; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

void Pliers::analysisCB(const can_msgs::Finish::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  // ROS_WARN_STREAM("Pliers; FINISH : state "<< act.isActive());

  if (!act.isActive() || msg->val != PLIERS)
    return;

  if (!fifo.empty()) {
    // ROS_INFO_STREAM("Pliers; FiFo : not empty");
    sendMsg();

  } else {
    procedures_msgs::PliersResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  }
}

inline void Pliers::sendMsg() {
  can_msgs::ActionPliers msg;
  //direction
  while (!fifo.empty()) {
    msg.action = fifo.front().action;
    msg.level = fifo.front().level;
    ARDUINO_pliers_pub.publish(temp);

    fifo.erase(fifo.begin());
  }
}

void Pliers::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    fifo.clear();
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pliers_procedure");

  Pliers pliering(ros::this_node::getName());
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
