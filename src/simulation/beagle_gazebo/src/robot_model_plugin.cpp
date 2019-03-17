#include "beagle_gazebo/robot_model_plugin.hpp"

using namespace gazebo;

void RobotModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    this->model = _parent;

    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "beagle_gazebo", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->nh.reset(new ros::NodeHandle("beagle_gazebo"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions subOptions = ros::SubscribeOptions::create<std_msgs::Float32>(
        Topics.STM_GO_TO,
        1,
        boost::bind(&RobotModelPush::OnRosMsg, this, _1),
        ros::VoidPtr(),
        &this->rosQueue
    );
    this->rosSub = this->nh->subscribe(subOptions);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&RobotModelPush::QueueThread, this));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RobotModelPush::OnUpdate, this));
    
}

void RobotModelPush::OnUpdate() {
    // Get model position
    const ignition::math::Pose3d& pose = this->model->WorldPose();

    // Publish it
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void RobotModelPush::OnRosMsg(const interface_msgs::Point &_msg) {
  //this->model->SetVelocity(_msg->data);
}

/// \brief ROS helper function that processes messages
void RobotModelPush::QueueThread() {
  static const double timeout = 0.01;
  while (this->nh->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}