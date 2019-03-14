#ifndef GAZEBO_ROS_POSITION_SENSOR_HPP
#define GAZEBO_ROS_POSITION_SENSOR_HPP

#include <functional>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "interface_msgs/Point.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

// From [http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin] and
// [http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6]
namespace gazebo {
  class RobotModelPush : public ModelPlugin {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;

    // Called by the world update start event
    void OnUpdate();

    // Handle an incoming message from ROS
    void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);

  private:
    // ROS helper function that processes messages
    void QueueThread();

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Ros objects
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber rosSub;

    // A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotModelPush)
}
#endif