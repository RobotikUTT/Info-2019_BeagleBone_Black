#!/usr/bin/python

## @file RobotServices.py
##    @brief Service interface for node readiness for python.
##	
##	
##    @author Alexis CARE
##
import rospy
from ai_msgs.srv import NodeReadiness

ROBOT_SRV = "/ai/node_watcher/node_readiness"
TIMEOUT = 20.0

##
## @brief      service function
##
## @param[in]  name_space  The name space of the node
## @param[in]  package     The package name
## @param[in]  ready       Node ready
## @param[in]  error_code  The error code if nessesary
##
##
def service_ready(namespace, package, ready, error_code = 0):
	node_name = "/{}/{}".format(namespace, package)
	try:
		rospy.wait_for_service(ROBOT_SRV)
		_ready_srv = rospy.ServiceProxy(ROBOT_SRV, NodeReadiness)

		if ready:
			rospy.loginfo("Node '{}' initialized".format(node_name))
			_ready_srv(node_name, ready, 0)
		else :
			rospy.logerr("Node '{}' not initialized".format(node_name))
			_ready_srv(node_name, ready, error_code)
	except Exception as e:
		rospy.logerr("Node '{}' could not contact {} server".format(node_name, ROBOT_SRV))
