#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "action_manager/AtomicAction.hpp"
#include "action_manager/PerformClient.hpp"
#include "action_manager/Point.hpp"

#include <string>
#include <sstream>

#include "scheduler/test/SampleActions.hpp"

// Static content for tests
class PerformingFixture : public ::testing::Test, public PerformClient {
protected:
  bool finished;
  bool paused;
  MessageActionPerformer msgPerf;
  AtomicAction messageAction;
  std::string received;
  ros::Subscriber sub;

  // Setup
  PerformingFixture() : PerformClient(), msgPerf(),
    messageAction("simple message action", "test_message") {
      
    reset();

    messageAction.addArg(make_arg("content", 45.5));

    //sub = nh.subscribe("test_messages", 3, &PerformingFixture::messageCallback, this);
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

  /*void messageCallback(const std_msgs::String& msg) {
    received = msg.data;
  }*/

  ai_msgs::Argument make_arg(std::string name, double value) {
    ai_msgs::Argument arg;
    arg.name = name;
    arg.value = value;

    return arg;
  }
};

/* 
 *  Test action point computation
 */
TEST_F(PerformingFixture, actionPointComputation) {
  Point robotPosition(30, 30);
  ROS_DEBUG("CALLING ADEQUATE SERVICE");
  // Assert we receive the right computed value
  ASSERT_EQ(
    ActionPoint(robotPosition, Point(2*robotPosition.x, 2*robotPosition.y)),
    messageAction.actionPoint(robotPosition)
  ) << "compute right action point";
}

/*
 *  Test action performing
 */
TEST_F(PerformingFixture, performing) {
  // Perform action
  reset();

  performAction(messageAction, Point(30, 30));

  auto response =
    ros::topic::waitForMessage<std_msgs::String>("test_messages", nh);
    
  ASSERT_EQ(response->data, std::string("got: 45.5"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "performing_test");
    
    return RUN_ALL_TESTS();
}
