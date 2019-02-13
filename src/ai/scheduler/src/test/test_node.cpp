#include "scheduler/test/test_node.hpp"

TestActionPerformer::TestActionPerformer() : ActionPerformer("test") {
    messagePub = nh.advertise<std_msgs::String>(MESSAGE_TOPIC, 3);
}

ActionPoint TestActionPerformer::computeActionPoint(std::vector<ai_msgs::Argument>* actionArgs, Point robot_pos) {
    // The robot teleports at twice it's coordinates
    return ActionPoint(robot_pos, Point(2*robot_pos.x, 2*robot_pos.y));
}

void TestActionPerformer::start() {
    std_msgs::String msg;
    std::ostringstream output;
    output << "got: ";
    output << getArg("content");
    msg.data = output.str();

    messagePub.publish(msg);

    // Action is finished
    actionPerformed();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_action");

  TestActionPerformer moving();
  ros::spin();
  return 0;
}
