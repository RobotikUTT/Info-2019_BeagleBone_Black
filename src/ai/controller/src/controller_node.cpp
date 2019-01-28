/** @file controller_node.cpp
*    @brief Node class wich controls all other nodes other robot watcher.
*    
*    @author Alexis CARE
*/

#include "controller/controller_node.hpp"

//define typedef for lisibility
typedef boost::shared_ptr<::procedures_msgs::MoveResult const> MoveResultConstPtr;

/**
 * @brief      Constructs the object.
 *
 * @param      n     NodeHandle var
 * 
 */
Controller::Controller(ros::NodeHandle *n) : move_action_client("/procedures/move_action", true)
{
  //attributes
  nh = *n;
  side = SIDE_GREEN;
  direction = NONE;
  emergency_stop = false;
  panelUp = 0;
  points_done = 0;

  //advertiser
  emergency_stop_pub = nh.advertise<ai_msgs::EmergencyStop>("emergency", 1);
  STM_SetPose_pub = nh.advertise<can_msgs::Point>("/STM/SetPose", 1);
  STM_AsserManagement_pub = nh.advertise<can_msgs::Status>("/STM/AsserManagement", 1);
  PANEL_Point_pub = nh.advertise<std_msgs::Int8>("/PANEL/AddPoint", 1);
  points_pub = nh.advertise<ai_msgs::PointsScored>("ai/points_scored", 1);

  //subscriber
  status_sub = nh.subscribe("robot_watcher/robot_status", 1, &Controller::setRobotStatus, this);
  robot_pos_sub = nh.subscribe("/STM/Position", 1, &Controller::setRobotPosition, this);
  robot_speed_sub = nh.subscribe("/STM/GetSpeed", 1, &Controller::setRobotSpeed, this);
  side_sub = nh.subscribe("side", 1, &Controller::setSide, this);
  nodes_status_sub = nh.subscribe("robot_watcher/nodes_status", 1, &Controller::checkForPanel, this);
  sonar_distance_sub = nh.subscribe("/ARDUINO/SonarDistance", 1, &Controller::processSonars, this);
  robot_blocked_sub = nh.subscribe("/STM/RobotBlocked", 1, &Controller::processRobotBlocked, this);

  // Services
  action_done_client = nh.serviceClient<ai_msgs::CurrentActionDone>("scheduler/currentActionDone");
  action_todo_client = nh.serviceClient<ai_msgs::GetActionToDo>("scheduler/actionToDo");
  action_todo_client.waitForExistence();

  move_action_client.waitForServer();

  service_ready("ai", "controller", 1);
}

void Controller::setSide(const ai_msgs::SetSide::ConstPtr &msg)
{
  side = msg->side;
}

// Set robot position inside controller
void Controller::setRobotPosition(const can_msgs::Point::ConstPtr &msg)
{
  // ROS_INFO_STREAM("robot pos_x: " << msg->pos_x
  //  << " robot pos_y: " << msg->pos_y
  //  << " robot angle: " << msg->angle);
  robot_pos_x = msg->pos_x;
  robot_pos_y = msg->pos_y;
  robot_angle = msg->angle;
}

/**
 * @brief      Gets the robot status and act accordingly
 *              - ROBOT_READY   : Nothing
 *              - ROBOT_RUNNING : wake up the robot and call SetAction methode
 *              - ROBOT_HALT    : halt all compute
 *
 * @param[in]  msg   The RobotStatus message
 */
void Controller::setRobotStatus(const ai_msgs::RobotStatus::ConstPtr &msg)
{
  robot_status = msg->robot_status;

  if (robot_status == ROBOT_READY)
  {
    //init card
    // ROS_DEBUG("Robot Ready");
    // std_msgs::Int8 msg;
    // msg.data = 1;

    //ARDUINO_Move_Pliers_pub.publish(msg);
  }
  else if (robot_status == ROBOT_RUNNING)
  {
    // ROS_DEBUG("Robot Running");
    int x, y, angle;
    nh.getParam("controller/robot_pos/x", x);
    nh.getParam("controller/robot_pos/y", y);
    nh.getParam("controller/robot_pos/angle", angle);

    if (side)
    { //if orange
      y = (1500 - y) + 1500;
      angle = -angle;
    }

    //init pos STM
    can_msgs::Point msg;

    robot_pos_x = x;
    robot_pos_y = y;
    robot_angle = angle;

    msg.pos_x = x;
    msg.pos_y = y;
    msg.angle = angle;

    STM_SetPose_pub.publish(msg);
    can_msgs::Status msg2;
    msg2.value = START;
    STM_AsserManagement_pub.publish(msg2);

    setAction();
  }
  else if (robot_status == ROBOT_HALT)
  {

    move_action_client.cancelAllGoals();

    can_msgs::Status msg;
    msg.value = STOP;
    STM_AsserManagement_pub.publish(msg);

    msg.value = RESET_ORDERS;
    STM_AsserManagement_pub.publish(msg);
  }
}

/**
 * @brief      check if the LED panel is connected
 *
 * @param[in]  msg   The NodesStatus message
 */
void Controller::checkForPanel(const ai_msgs::NodesStatus::ConstPtr &msg)
{
  for (int i = 0; i < msg->nodes_ready.size(); i++)
  {
    if (msg->nodes_ready[i] == "/board/PANEL")
    {
      panelUp = 1;
      nodes_status_sub.shutdown();
      sendPoint();
    }
  }
}

/**
 * @brief      Sends points to the LED panel
 */
void Controller::sendPoint()
{
  // ROS_INFO_STREAM("sendPoint point " << points_done);
  std_msgs::Int8 msg;
  msg.data = points_done;
  PANEL_Point_pub.publish(msg);
  points_done = 0;
}

/**
 * @brief      Gets the robot speed.
 *
 * @param[in]  msg   The CurrSpeed message
 */
void Controller::setRobotSpeed(const can_msgs::CurrSpeed::ConstPtr &msg)
{
  int16_t linearSpeed = msg->linear_speed;
  int16_t leftSpeed = msg->left_speed;
  int16_t rightSpeed = msg->right_speed;

  //  ROS_INFO_STREAM("SPEEDS|linear: " << linearSpeed
  //  << " left: " << leftSpeed
  //  << " right: " << rightSpeed);

  if (linearSpeed > 0)
  {
    direction = FORWARD;
  }
  else if (linearSpeed < 0)
  {
    direction = BACKWARD;
  }
  else
  {
    direction = NONE;
  }
}

/**
 * @brief      callback for every action message
 *
 * @param[in]  state    The action state
 * @param[in]  result   The action result
 *
 * @tparam     doneMsg  The message type
 */
template <class doneMsg>
void Controller::onActionDone(const actionlib::SimpleClientGoalState &state,
                            const doneMsg &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  //send point
  points_done += result->points_done;
  
  if (panelUp)
  {
    sendPoint();
  }

  if (robot_status != ROBOT_HALT)
  {
    //set old action to done
    ai_msgs::CurrentActionDone srv;
    srv.request.done = result->done;
    // ROS_INFO_STREAM("Action in state " << result->done);
    if (!action_done_client.call(srv))
    {
      ROS_ERROR("Failed to call service currentActionDone");
    }

    setAction();
  }
}

/**
 * @brief      Sets the action.to execute and call the action server associated
 */
void Controller::setAction()
{
  ai_msgs::GetActionToDo srv;
  srv.request.robot_pos_x = robot_pos_x;
  srv.request.robot_pos_y = robot_pos_y;

  // retrieve action to do
  if (action_todo_client.call(srv))
  {
    action_val = srv.response.action_val;
    // ROS_DEBUG("ACtion value: %d", action_val);
  }
  else
  {
    ROS_ERROR("Failed to call service GetActionToDo");
    action_val = -1;
  }

  if (action_val == 0) // TODO rework
  {
    // ROS_WARN("Action MOVE");
    procedures_msgs::MoveGoal goal;
    goal.points.push_back(srv.response.point);
    move_action_client.sendGoal(goal, boost::bind(&Controller::onActionDone<MoveResultConstPtr>, this, _1, _2));
    // ROS_WARN("Action MOVE send");
  }
  
}

/**
 * @brief      check the sonars data
 *
 * @param[in]  msg   The SonarDistance message
 */
void Controller::processSonars(const can_msgs::SonarDistance::ConstPtr &msg)
{
  bool last_emergency_value = emergency_stop;
  uint8_t front_left, front_right,
      back_left, back_right;

  front_left = msg->dist_front_left;
  front_right = msg->dist_front_right;
  back_left = msg->dist_back_left;
  back_right = msg->dist_back_right;

  // ROS_INFO_STREAM("DIST|" << front_left << "|" << front_right
  // << "|" << left << "|"  << right << "|" << back);
  /*ROS_INFO("DIST|%u|%u|%u|%u|",front_left,
    front_right, back_left, back_right);*/

  emergency_stop = false;
  if (direction == FORWARD)
  {
    if (front_left <= SONAR_MIN_DIST_FORWARD + 6 ||
        front_right <= SONAR_MIN_DIST_FORWARD + 16)
    {
      emergency_stop = true;
    }
  }
  else if (direction == BACKWARD)
  {
    if (back_left <= SONAR_MIN_DIST_BACKWARD ||
        back_right <= SONAR_MIN_DIST_BACKWARD)
    {
      emergency_stop = true;
    }
  }
  else
  {
    emergency_stop = false;
  }

  if (last_emergency_value != emergency_stop)
  {
    ai_msgs::EmergencyStop emergency_msg;
    emergency_msg.emergency_set = emergency_stop;
    emergency_stop_pub.publish(emergency_msg);

    if (emergency_stop)
    {
      ROS_WARN("SET EMG");
    }
    else
    {
      ROS_WARN("UNSET EMG");
    }

    can_msgs::Status can_msg;
    if (emergency_stop)
    {
      can_msg.value = SETEMERGENCYSTOP;
    }
    else
    {
      can_msg.value = UNSETEMERGENCYSTOP;
    }

    STM_AsserManagement_pub.publish(can_msg);
  }
}

/**
 * @brief      Callback to process a blocked robot
 *
 * @param[in]  msg   The RobotBlocked message
 * 
 * @todo To dev
 */
void Controller::processRobotBlocked(const can_msgs::RobotBlocked::ConstPtr &msg)
{
  ROS_WARN("Robot blocked");
}

/**
 * @brief      main function
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "controller_node");

  ros::NodeHandle nmh;

  Controller node(&nmh);

  ros::spin();
}
