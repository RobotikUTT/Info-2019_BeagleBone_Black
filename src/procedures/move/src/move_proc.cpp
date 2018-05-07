#include "move/move_proc.h"


Move::Move(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(   boost::bind(&Move::goalCB,    this));
    act.registerPreemptCallback(boost::bind(&Move::preemptCB, this));
    act.start();

    finish_sub        = nh.subscribe("/ALL/Finish",                    1, &Move::analysisCB,      this);
    robot_watcher_sub = nh.subscribe("/ai/robot_watcher/robot_status", 1, &Move::GetRobotStatus,  this);

    this->STMGoToAngle_pub        = nh.advertise<can_msgs::Point>("/STM/GoToAngle",         1);
    this->STMGoTo_pub             = nh.advertise<can_msgs::Point>("/STM/GoTo",              1);
    this->STMRotation_pub         = nh.advertise<can_msgs::Point>("/STM/Rotation",          1);
    this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo",  1);

    service_ready("procedure", "move", 1 );
  }

void Move::goalCB(){
  // ROS_WARN("Move: new goal");

  bool temp = !act.isActive();
  procedures_msgs::MoveGoal::ConstPtr msg = act.acceptNewGoal();
  for (int i = 0; i < msg->points.size(); i++) {
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].Opoint.x << "; y: " << msg->points[i].Opoint.y <<"; angle: "<< msg->points[i].Opoint.rot<< "; type: "<< (int)msg->points[i].type << "}" );
    fifo.push_back(MovePoint(msg->points[i].Opoint.x, msg->points[i].Opoint.y, msg->points[i].Opoint.rot, msg->points[i].type, msg->points[i].direction));
  }
  if(temp){
    sendMsg();
  }
}
void Move::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

void Move::analysisCB(const can_msgs::Finish::ConstPtr& msg){
  // ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

  if (!act.isActive() || msg->val != MOVE)
    return;

  if (!fifo.empty()) {
    sendMsg();
  } else {
    procedures_msgs::MoveResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  }
}

inline void Move::sendMsg() {
  can_msgs::Point msg;
  //direction
  while (!fifo.empty()) {
     ROS_INFO_STREAM("Point send: { x: " << fifo.front()._x << "; y: " << fifo.front()._y <<"; angle: "<< fifo.front()._angle<< "}" );
    switch (fifo.front()._move_type) {
      case GO_TO_ANGLE:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
        msg.angle = fifo.front()._angle;
        msg.direction = fifo.front()._direction;
        STMGoToAngle_pub.publish(msg);
        break;
      case GO_TO:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
        msg.direction = fifo.front()._direction;
        STMGoTo_pub.publish(msg);
        break;
      case ROTATION:
        msg.angle = fifo.front()._angle;
        STMRotation_pub.publish(msg);
        break;
      case ROTATION_NO_MODULO:
        msg.angle = fifo.front()._angle;
        STMRotationNoModulo_pub.publish(msg);
        break;
    }
    fifo.erase(fifo.begin());
    ros::Duration(0.01).sleep();
  }
}

void Move::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    fifo.clear();
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_procedure");

  Move moving(ros::this_node::getName());
  ros::spin();

  return 0;
}
