#include "move/move_proc.h"


Move::Move(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(boost::bind(&Move::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Move::preemptCB, this));
    act.start();

    sub = nh.subscribe("/ALL/Finish", 1, &Move::analysisCB, this);

    this->STMGoToAngle_pub = nh.advertise<can_msgs::Point>("/STM/GoToAngle", 1);
    this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
    this->STMRotation_pub = nh.advertise<can_msgs::Point>("/STM/Rotation", 1);
    this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo", 1);


    //subs
    // sub = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    service_ready("procedure", "move", 1 );

  }

void Move::goalCB()
{
  // ROS_WARN("Move: new goal");

  bool temp = !act.isActive();
  procedures_msgs::MoveGoal::ConstPtr msg = act.acceptNewGoal();
  for (int i = 0; i < msg->points.size(); i++) {
    // ROS_INFO_STREAM("Point["<< i <<"] recieved: { x: " << msg->points[i].end_x << "; y: " << msg->points[i].end_y <<"; angle: "<< msg->points[i].end_angle<< "; type: "<< (int)msg->points[i].type << "}" );
    fifo.push_back(MovePoint(msg->points[i].end_x, msg->points[i].end_y, msg->points[i].end_angle, msg->points[i].type));
  }

  if(temp){
    sendMsg();
  }
  //send point !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}
void Move::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

void Move::analysisCB(const can_msgs::Finish::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  // ROS_WARN_STREAM("Move; FINISH : state "<< act.isActive());

  if (!act.isActive() || msg->val != MOVE)
    return;

  if (!fifo.empty()) {
    /* code */
    // ROS_INFO_STREAM("Move; FiFo : not empty");
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
    // ROS_INFO_STREAM("Point send: { x: " << fifo.front()._x << "; y: " << fifo.front()._y <<"; angle: "<< fifo.front()._angle<< "}" );
    switch (fifo.front()._move_type) {
      case GO_TO_ANGLE:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
        msg.angle = fifo.front()._angle;
        STMGoToAngle_pub.publish(msg);
        break;
      case GO_TO:
        msg.pos_x = fifo.front()._x;
        msg.pos_y = fifo.front()._y;
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
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_procedure");

  Move moving(ros::this_node::getName());
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
