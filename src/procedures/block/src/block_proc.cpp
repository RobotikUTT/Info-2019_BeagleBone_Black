#include "block/block_proc.h"


Block::Block(std::string name):
  act(name,false),
  acM("/procedures/move_action", true)
  {
    act.registerGoalCallback(boost::bind(&Block::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Block::preemptCB, this));
    act.start();

    finish_sub = nh.subscribe("/ALL/Finish", 10, &Block::analysisCB, this);
    side_sub = nh.subscribe("/ai/side", 1, &Block::setSide, this );

    ARDUINO_pliers_pub = nh.advertise<can_msgs::ActionPliers>("/ARDUINO/ActionPliers", 10);

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
  ROS_WARN("Block: new goal");

  // bool temp = !act.isActive();
  phase = 0;
  point = 0;
  procedures_msgs::BlockGoal::ConstPtr msg = act.acceptNewGoal();
  // for (int i = 0; i < msg->points.size(); i++) {
  //   fifo.push_back();
  // }

  //MARCHE PAS OVERFLOW
  objectif = GroupBlocks(msg->block_action.x, msg->block_action.y, msg->block_action.rot, msg->depot.x, msg->depot.y, side);

  sendMsg();

  // if(temp){
  //   sendMsg();
  // }
  ROS_WARN("Block: end");

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

  if (!act.isActive() || msg->val != BLOCK)
    return;

  phase++;

  // if (!fifo.empty()) {
  //   /* code */
  //   ROS_INFO_STREAM("Block; FiFo : not empty");
  sendMsg();


  // } else {
  //   procedures_msgs::MoveResult result_;
  //   result_.done = 1;
  //   act.setSucceeded(result_);
  // }
}

void Block::sendMsg() {
  ROS_WARN_STREAM("phase: " << phase);

  switch (phase) {
    case 0:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      ROS_WARN_STREAM("msg 1");
      temp.end_x = objectif.proc_point[0].x;
      temp.end_y = objectif.proc_point[0].y;
      temp.end_angle = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      ROS_WARN_STREAM("msg 2");
      temp.end_x = objectif.proc_point[1].x;
      temp.end_y = objectif.proc_point[1].y;
      temp.end_angle = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      ROS_WARN_STREAM("msg 3");
      temp.end_x = objectif.proc_point[2].x;
      temp.end_y = objectif.proc_point[2].y;
      temp.end_angle = objectif.proc_point[2].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 1:{
      ROS_WARN_STREAM("msg 4");

      can_msgs::ActionPliers temp;
      temp.action = TAKE_BLOCK;
      temp.level = 0;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 2:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.proc_point[1].x;
      temp.end_y = objectif.proc_point[1].y;
      temp.end_angle = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[0].x;
      temp.end_y = objectif.proc_point[0].y;
      temp.end_angle = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[3].x;
      temp.end_y = objectif.proc_point[3].y;
      temp.end_angle = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[4].x;
      temp.end_y = objectif.proc_point[4].y;
      temp.end_angle = objectif.proc_point[4].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 3:{
      can_msgs::ActionPliers temp;
      temp.action = TAKE_BLOCK;
      temp.level = 1;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 4:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.proc_point[3].x;
      temp.end_y = objectif.proc_point[3].y;
      temp.end_angle = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.depos.x;
      temp.end_y = objectif.depos.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 5:{
      can_msgs::ActionPliers temp;
      temp.action = RELEASE_BLOCK;
      temp.level = 0;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 6:{
      point += 3;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[5].x;
      temp.end_y = objectif.proc_point[5].y;
      temp.end_angle = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[6].x;
      temp.end_y = objectif.proc_point[6].y;
      temp.end_angle = objectif.proc_point[6].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[7].x;
      temp.end_y = objectif.proc_point[7].y;
      temp.end_angle = objectif.proc_point[7].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 7:{
      can_msgs::ActionPliers temp;
      temp.action = TAKE_BLOCK;
      temp.level = 0;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 8:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.proc_point[6].x;
      temp.end_y = objectif.proc_point[6].y;
      temp.end_angle = objectif.proc_point[6].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[5].x;
      temp.end_y = objectif.proc_point[5].y;
      temp.end_angle = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.depos.x;
      temp.end_y = objectif.depos.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 9:{
      can_msgs::ActionPliers temp;
      temp.action = RELEASE_BLOCK;
      temp.level = 3;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 10:{
      point += 3;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[3].x;
      temp.end_y = objectif.proc_point[3].y;
      temp.end_angle = objectif.proc_point[3].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.end_x = objectif.proc_point[8].x;
      temp.end_y = objectif.proc_point[8].y;
      temp.end_angle = objectif.proc_point[8].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 11:{
      can_msgs::ActionPliers temp;
      temp.action = TAKE_BLOCK;
      temp.level = 0;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 12:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.proc_point[9].x;
      temp.end_y = objectif.proc_point[9].y;
      temp.end_angle = objectif.proc_point[9].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 13:{
      can_msgs::ActionPliers temp;
      temp.action = TAKE_BLOCK;
      temp.level = 1;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 14:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.end_x = objectif.depos.x;
      temp.end_y = objectif.depos.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 15:{
      can_msgs::ActionPliers temp;
      temp.action = RELEASE_BLOCK;
      temp.level = 4;
      ARDUINO_pliers_pub.publish(temp);
      break;
    }
    case 16:{
      point += 9;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.end_x = objectif.depos_.x;
      temp.end_y = objectif.depos_.y;
      temp.end_angle = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneMove, this, _1, _2));
      break;
    }
    case 17:{
      procedures_msgs::BlockResult result_;
      result_.done = 1;
      result_.points_done = point;
      act.setSucceeded(result_);
    }
    default: {
      ROS_WARN_STREAM("default");
      //error
    }
  }
  ROS_WARN_STREAM("end switch");


  // fifo.erase(fifo.begin());
}

void Block::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

void Block::DoneMove( const actionlib::SimpleClientGoalState& state, const MoveResultConstPtr & result){
  point += result->points_done;

  phase ++;

  sendMsg();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_procedure");

  Block blkAction(ros::this_node::getName());
  ros::spin();

  return 0;
}
