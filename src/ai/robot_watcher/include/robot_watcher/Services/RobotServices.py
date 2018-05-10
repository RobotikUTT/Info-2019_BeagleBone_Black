#!/usr/bin/python

import rospy
from ai_msgs.srv import NodeReadiness

ROBOT_SRV = "/ai/robot_watcher/node_readiness"
TIMEOUT = 20.0

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
		rospy.logerr("Node '{}' could not contacte {} server".format(node_name, ROBOT_SRV))
