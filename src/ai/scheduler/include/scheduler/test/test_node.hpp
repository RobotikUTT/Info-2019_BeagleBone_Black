#ifndef TEST_NODE_HPP
#define TEST_NODE_HPP

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "std_msgs/String.h"

#include "action_manager/ActionPerformer.hpp"

#include "action_manager/Point.hpp"

#include <iostream>
#include <string>
#include <sstream>

const std::string MESSAGE_TOPIC = "/test/action_response";

// Example test performer
class TestActionPerformer : public ActionPerformer {
	ros::Publisher messagePub;

public:
	TestActionPerformer();

	ActionPoint computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, Point robot_pos) override;
	void start() override;
};

#endif