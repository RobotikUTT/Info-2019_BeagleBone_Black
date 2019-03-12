#ifndef GAZEBO_ROS_POSITION_SENSOR_HPP
#define GAZEBO_ROS_POSITION_SENSOR_HPP

#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Vector3.hh>

// From [http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin]
namespace gazebo {
  class RobotModelPush : public ModelPlugin {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;

    // Called by the world update start event
    void OnUpdate();

  
  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotModelPush)
}
#endif