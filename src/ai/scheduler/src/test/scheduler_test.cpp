#include <gtest/gtest.h>
#include <ros/ros.h>

#include "std_msgs/String.h"

#include "action_manager/ActionPerformer.hpp"
#include "action_manager/AtomicAction.hpp"
#include "action_manager/PerformClient.hpp"
#include "action_manager/Point.hpp"

#include <iostream>
#include <string>
#include <sstream>

// Static content for tests
class SchedulerFixture : public ::testing::Test {
protected:
	// Setup
	SchedulerFixture() {
		
	}

	ai_msgs::Argument make_arg(std::string name, double value) {
		ai_msgs::Argument arg;
		arg.name = name;
		arg.value = value;

		return arg;
	}
};

/* 
 *	Test action point computation
 */
TEST_F(SchedulerFixture, none) {
	
}


int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "scheduler_test");

	return RUN_ALL_TESTS();
}
