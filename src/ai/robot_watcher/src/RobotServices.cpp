#include <robot_watcher/Services/RobotServices.h>

using namespace std;

string ROBOT_SRV = "/ai/robot_watcher/node_readiness";
float TIMEOUT = 10.0;


void service_ready(const string name_space, const string package, const bool ready, const uint8_t error_code){
	string node_name = "/" + name_space + "/" + package;

	try {
		if (!ros::service::waitForService(ROBOT_SRV, ros::Duration(TIMEOUT)))
			throw;
		ros::NodeHandle nh;
		ros::ServiceClient readyPub = nh.serviceClient<ai_msgs::NodeReadiness>(ROBOT_SRV);
        ai_msgs::NodeReadiness msg;
        msg.request.ready = ready;
        msg.request.node_name = node_name;

        if (ready){
            ROS_INFO_STREAM("Node " << node_name << " initialized.");
						msg.request.error_code = 0;
				} else{
            ROS_ERROR_STREAM("Node " << node_name << " not initialized.");
						msg.request.error_code = error_code;
					}
				if (!readyPub.call(msg))
						throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("status_services for '"<< node_name <<"' couldn't contact ai/game_status to send init notification.");
    }
}
