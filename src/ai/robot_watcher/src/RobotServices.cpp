#include <robot_watcher/Services/RobotServices.h>

using namespace std;

string ROBOT_SRV = "/ai/robot_watcher/node_readiness";
float TIMEOUT = 5.0;


void service_ready(const string name_space, const string package, const bool val){
	string node_name = "/" + name_space + "/" + package;

	try {
		if (!ros::service::waitForService(ROBOT_SRV, ros::Duration(TIMEOUT)))
			throw;
		ros::NodeHandle nh;
		ros::ServiceClient readyPub = nh.serviceClient<robot_watcher::NodeReadiness>(ROBOT_SRV);
        robot_watcher::NodeReadiness msg;
        msg.request.ready = val;
        msg.request.node_name = node_name;
        if (!readyPub.call(msg))
            throw;
        if (val)
            ROS_INFO_STREAM("Node " << node_name << " initialized.");
        else
            ROS_ERROR_STREAM("Node " << node_name << " not initialized.");
    }
    catch(...)
    {
        ROS_ERROR_STREAM("status_services for '"<< node_name <<"' couldn't contact ai/game_status to send init notification.");
    }
}