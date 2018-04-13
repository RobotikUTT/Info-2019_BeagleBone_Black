#!/usr/bin/python

import rospy
from ai_msgs.srv import NodeReadiness

# class RobotStatus(object):
# 	"""docstring for RobotStatus"""
# 	ROBOT_SRV = "/ai/robot_watcher/node_readiness"
# 	TIMEOUT = 5.0
# 	def __init__(self, namespace, package):
# 		self.node_name = "/{}/{}".format(namespace, package)

# 	def ready(self, val):
# 		try:
# 			rospy.wait_for_service(RobotStatus.ROBOT_SRV, timeout = RobotStatus.TIMEOUT)
# 			_ready_srv = rospy.ServiceProxy(RobotStatus.ROBOT_SRV, NodeReadiness)
# 			_ready_srv(self.node_name, val)

# 			if val:
# 				rospy.loginfo("Node '{}' initialized".format(self.node_name))
# 			else :
# 				rospy.logerr("Node '{}' not initialized".format(self.node_name))
# 		except Exception as e:
# 			rospy.logerr("Node '{}' could not contacte {} server".format(self.node_name, RobotStatus.ROBOT_SRV))

ROBOT_SRV = "/ai/robot_watcher/node_readiness"
TIMEOUT = 5.0

def service_ready(namespace, package, val):
	node_name = "/{}/{}".format(namespace, package)
	try:
		rospy.wait_for_service(ROBOT_SRV, timeout = TIMEOUT)
		_ready_srv = rospy.ServiceProxy(ROBOT_SRV, NodeReadiness)
		_ready_srv(node_name, val)

		if val:
			rospy.loginfo("Node '{}' initialized".format(node_name))
		else :
			rospy.logerr("Node '{}' not initialized".format(node_name))
	except Exception as e:
		rospy.logerr("Node '{}' could not contacte {} server".format(node_name, ROBOT_SRV))
