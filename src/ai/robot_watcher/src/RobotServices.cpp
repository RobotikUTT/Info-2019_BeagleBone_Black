/** @file RobotServices.cpp
*	@brief Service interface for node readiness.
*	
*	
*	@author Alexis CARE
*/
#include <robot_watcher/Services/RobotServices.h>

using namespace std;

const float TIMEOUT = 20.0;


/**
 * @brief function used to declare a node as ready
 *
 * @param[in]  name_space  The name space of the node
 * @param[in]  package     The package name
 * @param[in]  ready       Node ready
 * @param[in]  error_code  The error code if nessesary
 * 
 */
void service_ready(const string name_space, const string package, const int state, const uint8_t error_code /*= 0*/) {
	string node_name = "/" + name_space + "/" + package;

	ROS_WARN_STREAM("Deprecated use of robot_watcher services functions for " << node_name << "");

	try {
		if (!ros::service::waitForService(Topics::NODE_WATCHER_SERVICE)) {
			throw;
		}

		ros::NodeHandle nh;
		ros::ServiceClient readyPub = nh.serviceClient<ai_msgs::NodeReadiness>(Topics::NODE_WATCHER_SERVICE);

		ai_msgs::NodeReadiness msg;
		msg.request.status.state_code = state;
		msg.request.node_name = node_name;
		msg.request.status.error_code = error_code;

		if (!readyPub.call(msg)) {
			throw;
		}
	} catch(...) {
		ROS_ERROR_STREAM("status_services for '"<< node_name <<"' couldn't contact node_watcher");
	}
}
