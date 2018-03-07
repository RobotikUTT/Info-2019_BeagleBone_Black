#!/usr/bin/python

import time
import rospy

# import msgs/svrs

from ai_robot_status.msg import NodesStatus, RobotStatus
from ai_robot_status.srv import NodeReadiness, NodeReadinessResponse

 # SRV:
 # GameStart: bool

class Status(object):
	"""docstring for status
	class for set robot status"""
	ROBOT_INIT     = 0 # nodes initializing
	ROBOT_READY    = 1 # all nodes ready, waiting start
	ROBOT_RUNNING  = 2 # robot in game
	ROBOT_HALT     = 3 # end of game

	NODES_INIT    = 0 # nodes initializing
	NODES_RUNNING = 1 # nodes running
	NODES_ERROR   = 2 # at least 1 node error

	NODES_CHECKLIST = {

	"/namespace/pkg" 	: None,
	"/sender/" 			: None, #test
	"/receiver/" 		: None  #test

	}


class RobotWatcher(object):
	"""docstring for RobotStatus
	node to monitor robot nodes"""
	INIT_TIMEOUT = 15 #sec

	def __init__(self):
		rospy.init_node("ai_robot_watcher", log_level=rospy.INFO)

		rospy.Service("/ai/robot_watcher/node_readiness", NodeReadiness, self.set_readiness)
		self._robot_status_publisher = rospy.Publisher("/ai/robot_watcher/robot_status", RobotStatus, queue_size = 1)
		self._nodes_status_publisher = rospy.Publisher("/ai/robot_watcher/nodes_status", NodesStatus, queue_size = 1)

		self.robot_status = Status.ROBOT_INIT
		self.nodes_status = Status.NODES_INIT

		self.init_start_time = time.time()

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			if self.robot_status == Status.ROBOT_INIT:
				self.check_nodes()
				if time.time() - self.init_start_time > RobotWatcher.INIT_TIMEOUT:
					if len([n for n in Status.NODES_CHECKLIST if Status.NODES_CHECKLIST[n] in [None, False]]) > 0:
						self.change_nodes_status(Status.NODES_ERROR)
					else:
						self.change_nodes_status(Status.NODES_RUNNING)
				if self.nodes_status == Status.NODES_RUNNING:
					self.change_robot_status(Status.ROBOT_READY)

			# elif self.robot_status == Status.ROBOT_READY:
			# 	pass

			# elif self.robot_status == Status.ROBOT_RUNNING:
			# 	pass

			# else: # self.robot_status == Status.ROBOT_HALT:
			# 	pass

			self.publish_robot_status()
			self.publish_nodes_status()

			r.sleep()


	def check_nodes(self):
		for _, val in Status.NODES_CHECKLIST.items():
			if val == None or val == False:
				return
		if self.nodes_status == Status.NODES_INIT:
			self.change_nodes_status(Status.NODES_RUNNING)

	def publish_robot_status(self):
		msg = RobotStatus(self.robot_status, self.nodes_status)
		self._robot_status_publisher.publish(msg)

	def publish_nodes_status(self):
		msg = NodesStatus()
		for node, val in Status.NODES_CHECKLIST.items():
			if val == None:
				msg.nodes_init.append(node)
			elif val == True:
				msg.nodes_ready.append(node)
			else:
				msg.nodes_error.append(node)
		self._nodes_status_publisher.publish(msg)

	def change_robot_status(self, status):
		if status == Status.ROBOT_READY:
			rospy.loginfo("All node started: Robot standing by")
		elif status == Status.ROBOT_RUNNING:
			rospy.loginfo("Game started: Robot running")
		elif status == Status.ROBOT_HALT:
			rospy.loginfo("Game stoped: Robot halted")
		self.robot_status = status

	def change_nodes_status(self, status):
		if status == Status.NODES_RUNNING:
			rospy.loginfo("All Nodes ready")
		elif status == Status.NODES_ERROR :
			rospy.logerr("Nodes init not complete\nnodes not ready '{}'\nnodes unresponsive '{}"
				.format(
					str([n for n in Status.NODES_CHECKLIST if Status.NODES_CHECKLIST[n] == False]),
					str([n for n in Status.NODES_CHECKLIST if Status.NODES_CHECKLIST[n] == None])))
		self.nodes_status = status

	def set_readiness(self, msg):
		if msg.node_name in Status.NODES_CHECKLIST:
			Status.NODES_CHECKLIST[msg.node_name] = msg.ready
			return NodeReadinessResponse()
		else: 
			rospy.logwarn("Node {} not in cheklist".format(msg.node_name))
			return NodeReadinessResponse()


if __name__ == '__main__':
	RobotWatcher()
			

