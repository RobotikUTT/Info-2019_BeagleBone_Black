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

// Example test performer
class MessageActionPerformer : public ActionPerformer {
	ros::Publisher messagePub;

public:
	MessageActionPerformer() : ActionPerformer("test_message") {
		messagePub = nh.advertise<std_msgs::String>("/test_messages", 3);
	}

	ActionPoint computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, Point robot_pos) override {
		// The robot teleports at twice it's coordinates
		return ActionPoint(robot_pos, Point(2*robot_pos.x, 2*robot_pos.y));
	}
	
	void start() override {
		std_msgs::String msg;
		std::ostringstream output;
		output << "got: ";
		output << getArg("content");
		msg.data = output.str();

		messagePub.publish(msg);

		// Action is finished
		actionPerformed();
	}
};

// Static content for tests
class PerformingFixture : public ::testing::Test, public PerformClient {
protected:
	// Action state and result
	std::string received;
	bool finished;
	bool paused;

	// Tools
	AtomicAction messageAction;
	ros::Subscriber sub;
	MessageActionPerformer msgPerf;
	ros::AsyncSpinner spinner; // Use 4 threads
	
	// Setup
	PerformingFixture() : PerformClient(),
		messageAction("simple message action", "test_message"), msgPerf(), spinner(4) {
		
		// Init states
		reset();
		messageAction.addArg(make_arg("content", 45.5));

		// Subscribe to answers
		sub = nh.subscribe("test_messages", 3, &PerformingFixture::messageCallback, this);
	
		// Spin ros
		spinner.start();
	}

	void onFinished(const actionlib::SimpleClientGoalState& state,
		const ai_msgs::PerformResultConstPtr& result) {
		finished = true;
	}

	virtual void onPaused() {
		paused = true;
	}

	void reset() {
		finished = false;
		paused = false;
	}

	void messageCallback(const std_msgs::String& msg) {
		received = msg.data;
		finished = true;
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
TEST_F(PerformingFixture, actionPointComputation) {
	Point robotPosition(30, 30);
	
	// Assert we receive the right computed value
	ASSERT_EQ(
		ActionPoint(robotPosition, Point(2*robotPosition.x, 2*robotPosition.y)),
		messageAction.actionPoint(robotPosition)
	) << "compute right action point";
}

/*
 *	Test action performing
 */
TEST_F(PerformingFixture, performing) {
	// Perform action
	reset();
	PerformClient::performAction(messageAction, Point(30, 30));
	
	// Wait for message to be taken in account
	while (ros::ok() && !finished) {
		ros::spinOnce();
	}

	// Assert response is correct
	ASSERT_EQ(received, std::string("got: 45.5"));
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "performing_test");

	

	return RUN_ALL_TESTS();
}