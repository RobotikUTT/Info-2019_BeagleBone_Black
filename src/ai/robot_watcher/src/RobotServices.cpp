/** @file RobotServices.cpp
*	@brief Service interface for node readiness.
*	
*	
*	@author Alexis CARE
*/
#include <robot_watcher/Services/RobotServices.h>

using namespace std;

const string ROBOT_SRV = "/ai/robot_watcher/node_readiness";
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
void service_ready(const string name_space, const string package, const bool ready, const uint8_t error_code /*= 0*/) {
	string node_name = "/" + name_space + "/" + package;

	try {
		if (!ros::service::waitForService(ROBOT_SRV)) {
			throw;
		}

		ros::NodeHandle nh;
		ros::ServiceClient readyPub = nh.serviceClient<ai_msgs::NodeReadiness>(ROBOT_SRV);

		ai_msgs::NodeReadiness msg;
		msg.request.ready = ready;
		msg.request.node_name = node_name;

		if (ready) {
			ROS_INFO_STREAM("Node " << node_name << " initialized.");
			msg.request.error_code = 0;
		} else {
			ROS_ERROR_STREAM("Node " << node_name << " not initialized.");
			msg.request.error_code = error_code;
		}

		if (!readyPub.call(msg)) {
			throw;
		}
	} catch(...) {
		ROS_ERROR_STREAM("status_services for '"<< node_name <<"' couldn't contact ai/game_status to send init notification.");
	}
}
