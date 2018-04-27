#include "block/block_proc.h"


Block::Block(std::string name):
  act(name,false)
  {
    act.registerGoalCallback(boost::bind(&Block::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Block::preemptCB, this));
    act.start();

    sub = nh.subscribe("/ALL/Finish", 10, &Block::analysisCB, this);

    // this->STMGoToAngle_pub = nh.advertise<can_msgs::Point>("/STM/GoToAngle", 1);
    // this->STMGoTo_pub = nh.advertise<can_msgs::Point>("/STM/GoTo", 1);
    // this->STMRotation_pub = nh.advertise<can_msgs::Point>("/STM/Rotation", 1);
    // this->STMRotationNoModulo_pub = nh.advertise<can_msgs::Point>("/STM/RotationNoModulo", 1);


    //subs
    // sub = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
    service_ready("procedure", "block", 1 );

  }

void Block::goalCB()
{
  ROS_WARN("Move: new goal");

  bool temp = !act.isActive();
  procedures_msgs::MoveGoal::ConstPtr msg = act.acceptNewGoal();
  for (int i = 0; i < msg->points.size(); i++) {
    fifo.push_back();
  }

  if(temp){
    sendMsg();
  }

}
void Block::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

void Block::analysisCB(const can_msgs::Finish::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  ROS_WARN_STREAM("Block; FINISH : state "<< act.isActive());

  if (!act.isActive())
    return;

  if (!fifo.empty()) {
    /* code */
    ROS_INFO_STREAM("Block; FiFo : not empty");
    sendMsg();


  } else {
    procedures_msgs::MoveResult result_;
    result_.done = 1;
    act.setSucceeded(result_);
  }
}

inline void Move::sendMsg() {

  fifo.erase(fifo.begin());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_procedure");

  Move moving(ros::this_node::getName());
  ros::spin();

  return 0;
}
