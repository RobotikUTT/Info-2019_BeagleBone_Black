#include "procedures/move_proc.h"


Move::Move(std::string name):
  act(nh,name,false)
  {
    act.registerGoalCallback(boost::bind(&Move::goalCB, this));
    act.registerPreemptCallback(boost::bind(&Move::preemptCB, this));

    //subs
    // sub = nh_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
  }

void Move::goalCB()
{
  ROS_DEBUG("%s: new goal", action_name_.c_str());
  // reset helper variables

  // accept the new goal
  // goal_ = as_.acceptNewGoal()->samples;
}

void Move::preemptCB()
{
  ROS_DEBUG("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  as_.setPreempted();
}

void Move::analysisCB(const std_msgs::Float32::ConstPtr& msg)
{
  // make sure that the action hasn't been canceled
  if (!as_.isActive())
    return;

  // data_count_++;
  // feedback_.sample = data_count_;
  // feedback_.data = msg->data;
  // //compute the std_dev and mean of the data
  // sum_ += msg->data;
  // feedback_.mean = sum_ / data_count_;
  // sum_sq_ += pow(msg->data, 2);
  // feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
  // as_.publishFeedback(feedback_);
  //
  // if(data_count_ > goal_)
  // {
  //   result_.mean = feedback_.mean;
  //   result_.std_dev = feedback_.std_dev;
  //
  //   if(result_.mean < 5.0)
  //   {
  //     ROS_INFO("%s: Aborted", action_name_.c_str());
  //     //set the action state to aborted
  //     as_.setAborted(result_);
  //   }
  //   else
  //   {
  //     ROS_INFO("%s: Succeeded", action_name_.c_str());
  //     // set the action state to succeeded
  //     as_.setSucceeded(result_);
  //   }
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_procedure");

  Move moving(ros::this_node::getName());
  ros::spin();

  return 0;
}
