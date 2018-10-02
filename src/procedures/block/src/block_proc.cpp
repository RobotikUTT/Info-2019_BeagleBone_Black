/** @file block_proc.cpp
*    @brief The block action class
*    
*    @author Alexis CARE
*/
#include "block/block_proc.h"

/**
 * @brief      Constructs the object.
 *
 * @param[in]  name  The name of the  action server
 */
Block::Block(std::string name):
  act(name,false),
  acM("/procedures/move_action", true),
  acP("/procedures/pliers_action", true)
  {
    act.registerGoalCallback(   boost::bind(&Block::goalCB,     this));
    act.registerPreemptCallback(boost::bind(&Block::preemptCB,  this));
    act.start();

    side_sub = nh.subscribe("/ai/side", 1, &Block::setSide, this );

    service_ready("procedure", "block", 1 );

  }

/**
 * @brief      The new Goal callback
 */
void Block::goalCB(){
  // ROS_WARN("Block: new goal");

  phase = 0;
  point = 0;
  procedures_msgs::BlockGoal::ConstPtr msg = act.acceptNewGoal();

   ROS_WARN_STREAM("Block:  x: " << (int)msg->block_action.x << " y: " << (int)msg->block_action.y << " rot: " << (int)msg->block_action.rot << " xdep: " << (int)msg->depot.x << " ydep: " <<(int) msg->depot.y << " side: " << (int)side);

  objectif = GroupBlocks(msg->block_action.x, msg->block_action.y, msg->block_action.rot, msg->depot.x, msg->depot.y, side);

  sendMsg();
}

/**
 * @brief      The preempt Callback
 */
void Block::preemptCB(){
  ROS_DEBUG("Move; Preempted");
  // set the action state to preempted
  act.setPreempted();
}

/**
 * @brief      Sends all Block order in the correspondate phase to the Arduino.
 */
void Block::sendMsg() {

  if (!act.isActive())
      return;

  ROS_WARN_STREAM("phase: " << (int)phase);


  switch (phase) {
    case 0:{

      procedures_msgs::PliersGoal goal1;

      procedures_msgs::APliers temp1;

      temp1.action = SET_PLIERS;
      temp1.level = 1;

      goal1.act.push_back(temp1);


      acP.sendGoal(goal1);

      procedures_msgs::MoveGoal goal;
      procedures_msgs::MPoint temp;

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[1].x;
      temp.Opoint.y = objectif.proc_point[1].y;
      temp.Opoint.rot = objectif.proc_point[1].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[2].x;
      temp.Opoint.y = objectif.proc_point[2].y;
      temp.Opoint.rot = objectif.proc_point[2].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 1:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = TAKE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);


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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[0].x;
      temp.Opoint.y = objectif.proc_point[0].y;
      temp.Opoint.rot = objectif.proc_point[0].angle;
      temp.type = GO_TO;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[3].x;
      temp.Opoint.y = objectif.proc_point[3].y;
      temp.Opoint.rot = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[4].x;
      temp.Opoint.y = objectif.proc_point[4].y;
      temp.Opoint.rot = objectif.proc_point[4].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 3:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = TAKE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 5:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = RELEASE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[5].x;
      temp.Opoint.y = objectif.proc_point[5].y;
      temp.Opoint.rot = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[6].x;
      temp.Opoint.y = objectif.proc_point[6].y;
      temp.Opoint.rot = objectif.proc_point[6].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[7].x;
      temp.Opoint.y = objectif.proc_point[7].y;
      temp.Opoint.rot = objectif.proc_point[7].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 7:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = TAKE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);
      temp.action = SET_PLIERS;
      temp.level = 2;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[5].x;
      temp.Opoint.y = objectif.proc_point[5].y;
      temp.Opoint.rot = objectif.proc_point[5].angle;
      temp.type = GO_TO;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos_.x;
      temp.Opoint.y = objectif.depos_.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 9:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = RELEASE_BLOCK;
      temp.level = 2;

      goal.act.push_back(temp);

      temp.action = SET_PLIERS;
      temp.level = 1;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[3].x;
      temp.Opoint.y = objectif.proc_point[3].y;
      temp.Opoint.rot = objectif.proc_point[3].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.proc_point[8].x;
      temp.Opoint.y = objectif.proc_point[8].y;
      temp.Opoint.rot = objectif.proc_point[8].angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 11:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = TAKE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 13:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = TAKE_BLOCK;
      temp.level = 0;

      goal.act.push_back(temp);

      temp.action = SET_PLIERS;
      temp.level = 3;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
      goal.points.push_back(temp);

      temp.Opoint.x = objectif.depos.x;
      temp.Opoint.y = objectif.depos.y;
      temp.Opoint.rot = objectif.depos.angle;
      temp.type = GO_TO_ANGLE;
      temp.direction = DEFAULT;
      goal.points.push_back(temp);
      acM.sendGoal(goal, boost::bind(&Block::DoneAction<MoveResultConstPtr>, this, _1, _2));
      break;
    }
    case 15:{
      procedures_msgs::PliersGoal goal;

      procedures_msgs::APliers temp;

      temp.action = RELEASE_BLOCK;
      temp.level = 3;

      goal.act.push_back(temp);

      temp.action = SET_PLIERS;
      temp.level = 1;

      goal.act.push_back(temp);

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
      temp.direction = DEFAULT;
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
}

/**
 * @brief      Sets the robot side.
 *
 * @param[in]  msg   The SetSide message
 */
void Block::setSide(const ai_msgs::SetSide::ConstPtr& msg){
  ROS_WARN_STREAM("SET SIDE BLOCK");
  side = msg->side;
}

/**
 * @brief      The action done callback
 *
 * @param[in]  state    The state of the action
 * @param[in]  result   The result message
 *
 * @tparam     doneMsg  The action result type
 */
template <class doneMsg>
void Block::DoneActioDoneActionn( const actionlib::SimpleClientGoalState& state, const doneMsg & result){

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
    point += result->points_done;

    phase ++;

    sendMsg();
  }
}

/**
 * @brief      Gets the robot status.
 *
 * @param[in]  msg   The RobotStatus message
 */
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
