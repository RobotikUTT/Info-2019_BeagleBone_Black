#include "beagle_gazebo/robot_model_plugin.hpp"

void gazebo::RobotModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo::RobotModelPush::OnUpdate, this));
}

void gazebo::RobotModelPush::OnUpdate() {
    // Apply a small linear velocity to the model.
    this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
}