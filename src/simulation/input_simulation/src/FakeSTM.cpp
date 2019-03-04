#include "input_simulation/FakeSTM.hpp"

FakeSTM::FakeSTM() : nh() {
    // Publishers
    this->positionPub = nh.advertise<can_msgs::Point>("/STM/Position", 10);
    this->speedPub = nh.advertise<can_msgs::CurrSpeed>("/STM/GetSpeed", 10);

    this->finishedPub = nh.advertise<can_msgs::Finish>("/ALL/Finish", 10);
    this->robotBlockedPub = nh.advertise<can_msgs::RobotBlocked>("/STM/RobotBlocked", 10);

    // Subscribers
    this->speedSetterSub = nh.subscribe("/STM/Speed", 10, &FakeSTM::setSpeed, this);
    this->asserManagementSub = nh.subscribe("/STM/AsserManagement", 10, &FakeSTM::manage, this);
    
    // Basic orders
    this->goToAngleSub = nh.subscribe("/STM/GoToAngle", 10, &FakeSTM::goToAngle, this);
    this->goToSub = nh.subscribe("/STM/GoTo", 10, &FakeSTM::goTo, this);
    this->rotateSub = nh.subscribe("/STM/Rotation", 10, &FakeSTM::rotate, this);
    
    this->poseSetterSub = nh.subscribe("/STM/SetPose", 10, &FakeSTM::setPose, this);
}

void computePosition() {

}

void FakeSTM::addGoal(const can_msgs::Point::ConstPtr& msg, int type) {
    Goal goal;
    goal.point = *msg;
    goal.type = type;

    this->goals.push_back(goal);
}


// Setters
void FakeSTM::setSpeed(const can_msgs::Speed::ConstPtr& msg); {

}

void FakeSTM::manage(const can_msgs::STMStatus::ConstPtr& msg); {
    this->mode = msg->value;
}

void FakeSTM::goToAngle(const can_msgs::Point::ConstPtr& msg); {
    this->addGoal(msg, ORIENTED_POSITION);
}

void FakeSTM::goTo(const can_msgs::Point::ConstPtr& msg); {
    this->addGoal(msg, POSITION);
}

void FakeSTM::rotate(const can_msgs::Point::ConstPtr& msg); {
    this->addGoal(msg, POSITION);
}

void FakeSTM::setPose(const can_msgs::Point::ConstPtr& msg); {
    this->position = msg;
}

