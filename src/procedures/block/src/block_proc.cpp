#include "block/block_proc.h"


Block::Block(std::string name):
  act(name,false),
  acM("/procedures/move_action", true),
  acP("/procedures/pliers_action", true)
  {
    act.registerGoalCallback(boost::bind(&Block::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Block::preemptCB, this));
    act.start();

    // finish_sub = nh.subscribe("/ALL/Finish", 10, &Block::analysisCB, this);
    side_sub = nh.subscribe("/ai/side", 1, &Block::setSide, this );

    // ARDUINO_pliers_pub = nh.advertise<can_msgs::ActionPliers>("/ARDUINO/ActionPliers", 10);

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
  // ROS_WARN("Block: new goal");

  // bool temp = !act.isActive();
  phase = 0;
  point = 0;
  procedures_msgs::BlockGoal::ConstPtr msg = act.acceptNewGoal();
  // for (int i = 0; i < msg->points.size(); i++) {
  //   fifo.push_back();
  // }

  objectif = GroupBlocks(msg->block_action.x, msg->block_action.y, msg->block_action.rot, msg->depot.x, msg->depot.y, side);

  sendMsg();

  // if(temp){
  //   sendMsg();
  // }
  // ROS_WARN("Block: end");

}

void Block::preemptCB()
{
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

// void Block::analysisCB(const can_msgs::Finish::ConstPtr& msg)
// {
//   // make sure that the action hasn't been canceled
//   // ROS_WARN_STREAM("Block; FINISH : state "<< act.isActive());
//
//   if (!act.isActive() || msg->val != BLOCK)
//     return;
//
//   phase++;
//
//   // if (!fifo.empty()) {
//   //   /* code */
//   //   ROS_INFO_STREAM("Block; FiFo : not empty");
//   sendMsg();
//
//
//   // } else {
//   //   procedures_msgs::MoveResult result_;
//   //   result_.done = 1;
//   //   act.setSucceeded(result_);
//   // }
// }

void Block::sendMsg() {

  if (!act.isActive())
      return;

  ROS_WARN_STREAM("phase: " << (int)phase);


  switch (phase) {
    case 0:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[1].x;
      temp.Opoint.y = objectif.proc_point[1].y;
      temp.Opoint.rot = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[2].x;
      temp.Opoint.y = objectif.proc_point[2].y;
      temp.Opoint.rot = objectif.proc_point[2].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 1:{
      procedures_msgs::PliersGoal goal;
      goal.action = TAKE_BLOCK;
      goal.level = 0;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 2:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[1].x;
      temp.Opoint.y = objectif.proc_point[1].y;
      temp.Opoint.rot = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[3].x;
      temp.Opoint.y = objectif.proc_point[3].y;
      temp.Opoint.rot = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[4].x;
      temp.Opoint.y = objectif.proc_point[4].y;
      temp.Opoint.rot = objectif.proc_point[4].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 3:{
      procedures_msgs::PliersGoal goal;
      goal.action = TAKE_BLOCK;
      goal.level = 1;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 4:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[3].x;
      temp.Opoint.y = objectif.proc_point[3].y;
      temp.Opoint.rot = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 5:{
      procedures_msgs::PliersGoal goal;
      goal.action = RELEASE_BLOCK;
      goal.level = 0;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 6:{
      point += 3;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[5].x;
      temp.Opoint.y = objectif.proc_point[5].y;
      temp.Opoint.rot = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[6].x;
      temp.Opoint.y = objectif.proc_point[6].y;
      temp.Opoint.rot = objectif.proc_point[6].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[7].x;
      temp.Opoint.y = objectif.proc_point[7].y;
      temp.Opoint.rot = objectif.proc_point[7].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 7:{
      procedures_msgs::PliersGoal goal;
      goal.action = TAKE_BLOCK;
      goal.level = 0;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 8:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[6].x;
      temp.Opoint.y = objectif.proc_point[6].y;
      temp.Opoint.rot = objectif.proc_point[6].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[5].x;
      temp.Opoint.y = objectif.proc_point[5].y;
      temp.Opoint.rot = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 9:{
      procedures_msgs::PliersGoal goal;
      goal.action = RELEASE_BLOCK;
      goal.level = 3;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));

      break;
    }
    case 10:{
      point += 3;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[3].x;
      temp.Opoint.y = objectif.proc_point[3].y;
      temp.Opoint.rot = objectif.proc_point[3].angle;
      temp.type = GO_TO;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[8].x;
      temp.Opoint.y = objectif.proc_point[8].y;
      temp.Opoint.rot = objectif.proc_point[8].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 11:{
      procedures_msgs::PliersGoal goal;
      goal.action = TAKE_BLOCK;
      goal.level = 0;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 12:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[9].x;
      temp.Opoint.y = objectif.proc_point[9].y;
      temp.Opoint.rot = objectif.proc_point[9].angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 13:{
      procedures_msgs::PliersGoal goal;
      goal.action = TAKE_BLOCK;
      goal.level = 1;

      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 14:{
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 15:{
      procedures_msgs::PliersGoal goal;
      goal.action = RELEASE_BLOCK;
      goal.level = 4;
      acP.sendGoal(goal, boost::bind(&Block::DoneAction<PliersResultConstPtr>, this, _1, _2));
      break;
    }
    case 16:{
      point += 9;
      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 17:{
      procedures_msgs::BlockResult result_;
      result_.done = 1;
      result_.points_done = point;
      act.setSucceeded(result_);
      break;
    }
    default: {
      ROS_WARN_STREAM("default");
      //error
    }
  }
  // if(phase%2 == 0){
  //   ROS_INFO("MOVE");
  // } else {
  //   ROS_INFO("PLIERS");
  // }

  // fifo.erase(fifo.begin());
}

void Block::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  side = msg->side;
}

template <class doneMsg>
void Block::DoneAction( const actionlib::SimpleClientGoalState& state, const doneMsg & result){

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
    point += result->points_done;

    phase ++;

    sendMsg();
  }
}

void Block::GetRobotStatus(const ai_msgs::RobotStatus::ConstPtr& msg){
  if(msg->robot_watcher == ROBOT_HALT){
    act.shutdown();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_procedure");

  Block blkAction(ros::this_node::getName());
  ros::spin();

  return 0;
}
