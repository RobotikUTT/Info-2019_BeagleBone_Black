#include "pliers/pliers_proc.h"


Pliers::Pliers(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(   boost::bind(&Pliers::goalCB,    this));
    act.registerPreemptCallback(boost::bind(&Pliers::preemptCB, this));
    act.start();

    finish_sub        = nh.subscribe("/ALL/Finish",                    1, &Pliers::analysisCB,     this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Pliers::GetRobotStatus, this);

    ARDUINO_pliers_pub = nh.advertise<can_msgs::ActionPliers>("/ARDUINO/ActionPliers", 10);
    ARDUINO_Move_pliers_pub = nh.advertise<std_msgs::Int8>("/ARDUINO/MovePliers", 10);

    service_ready("procedure", "pliers", 1 );
  }

void Pliers::goalCB(){
  // ROS_WARN("Pliers: new goal");

  bool temp = !act.isActive();
  procedures_msgs::PliersGoal::ConstPtr msg = act.acceptNewGoal();
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].end_x << "; y: " << msg->points[i].end_y <<"; angle: "<< msg->points[i].end_angle<< "; type: "<< (int)msg->points[i].type << "}" );
  for (int i = 0; i < msg->act.size(); i++) {
    fifo.push_back(PliersCommand(msg->act[i].action, msg->act[i].level));
  }
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
  //direction
  while (!fifo.empty()) {
    ROS_INFO_STREAM("PLIERS SEND: action: " << (int)fifo.front().action << " level: " << (int)fifo.front().level);
    if (fifo.front().action == SET_PLIERS) {
      std_msgs::Int8 msg;
      msg.data = fifo.front().level;

      ARDUINO_Move_pliers_pub.publish(msg);

    } else {
      can_msgs::ActionPliers msg;
      msg.action = fifo.front().action;
      msg.level = fifo.front().level;
      ARDUINO_pliers_pub.publish(msg);
    }

    fifo.erase(fifo.begin());
    ros::Duration(0.01).sleep();
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
