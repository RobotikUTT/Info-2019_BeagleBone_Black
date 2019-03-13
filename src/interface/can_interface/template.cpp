/*
 * Following content is supposed to be parsed with python's format method
 * Therefore for an single {{ or }} in the resulting code, please use double
 * brackets {{ or }}, otherwise python will think it is a template content
 */
#include <ros/ros.h>

#include "node_watcher/NodeStatusHandler.hpp"

#include "can_msgs/Frame.h"

{includes}

using can_msgs::Frame;

class CanInterfaceNode {{
private:
	NodeStatusHandler nodes;
	ros::NodeHandle nh;

	// Can frames subscriber and published
	ros::Publisher can_pub;
	ros::Subscriber can_sub;

	// Generated variables
	{private_header}

public:
	// Constructor
	CanInterfaceNode(): nodes(), nh() {{
		// CAN publisher and subscriber
		this->can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);
		this->can_sub = nh.subscribe("received_messages", 100, &CanInterfaceNode::onCanFrame, this);

		// Other (generated pub/sub)
		{constructor_content}

		nodes.setNodeStatus("can_interface", "interface", NodeStatus::NODE_READY);
	}}

	// CAN to AI callback
	void onCanFrame(const can_msgs::Frame::ConstPtr& msg_from_can) {{
		if (msg_from_can->id == Frame::BBB_CAN_ADDR) {{
			switch (msg_from_can->data[0]) {{
				// First case
				case Frame::ORDER_WHOAMI:
					{{
						// Get associated board name
						std::string boardName;
						switch (msg_from_can->data[1]) {{
							// TODO auto generate this content
							case Frame::STM_CAN_ADDR:
								boardName = "STM";
								break;
							case Frame::ARDUINO_CAN_ADDR:
								boardName = "ARDUINO";
								break;
							case Frame::ZIGBEE_CAN_ADDR:
								boardName = "ZIGBEE";
								break;
							case Frame::PANEL_CAN_ADDR:
								boardName = "PANEL";
								break;
						}}
						
						// Retrieve status
						uint8_t status = msg_from_can->data[2];

						nodes.setNodeStatus(boardName, "board", status);
					}}
					break;
			
				// Other generated code
				{reception_switch_cases}
			}}
		}}
	}}

	// AI to CAN callbacks
	{public_header}
}};


int main(int argc, char **argv) {{
	ros::init(argc, argv, "can_interface_node");
	CanInterfaceNode node;
	ros::spin();
}}

		