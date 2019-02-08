#include <ros/ros.h>

#include "std_msgs/String.h"
#include "action_manager/ActionPerformer.hpp"

#include <sstream>

class MessageActionPerformer : public ActionPerformer {
    ros::Publisher messagePub;

public:
    MessageActionPerformer() : ActionPerformer("test_message") {
        messagePub = nh.advertise<std_msgs::String>("test_messages", 3);
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
