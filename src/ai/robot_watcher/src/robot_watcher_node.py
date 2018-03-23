#!/usr/bin/python
import time
import rospy

from robot_watcher.GPIOemulator.EmulatorGUI import GPIO
# import Adafruit_BBIO.GPIO as GPIO

# import msgs/svrs

from robot_watcher.msg import NodesStatus, RobotStatus
from robot_watcher.srv import NodeReadiness, NodeReadinessResponse
from robot_watcher.RStatus.State import RobotState, WatcherState


class RobotWatcherNode(object):
	"""docstring for RobotWatcherNode
	node to monitor robot nodes"""
	INIT_TIMEOUT 	= 15 #sec
	GAME_LENTH		= 10 #sec

	def __init__(self):

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup("P8_8",GPIO.IN, initial = GPIO.LOW)

		rospy.init_node("ai_robot_watcher", log_level=rospy.INFO)

		rospy.Service("/ai/robot_watcher/node_readiness", NodeReadiness, self.set_readiness)
		self._robot_watcher_publisher = rospy.Publisher("/ai/robot_watcher/robot_watcher", RobotStatus, queue_size = 1, latch=True)
		self._nodes_status_publisher = rospy.Publisher("/ai/robot_watcher/nodes_status", NodesStatus, queue_size = 1)

		self.robot_watcher 	= RobotState.ROBOT_INIT
		self.nodes_status 	= WatcherState.NODES_INIT
		self.pin 			= WatcherState.PIN_ON

		self.init_start_time = time.time()
		self.publish_robot_watcher()

		r = rospy.Rate(5)
		while not rospy.is_shutdown():

			if self.pin == WatcherState.PIN_ON:
				if GPIO.input("P8_8") == WatcherState.PIN_OFF:
					self.pin = WatcherState.PIN_OFF
					rospy.Timer(rospy.Duration(RobotWatcherNode.GAME_LENTH), self.halt_game, oneshot=True)

				
			if self.robot_watcher == RobotState.ROBOT_INIT:
				if self.nodes_ready():
					self.change_nodes_status(WatcherState.NODES_RUNNING)
				if time.time() - self.init_start_time > RobotWatcherNode.INIT_TIMEOUT:
					if len([n for n in WatcherState.NODES_CHECKLIST if WatcherState.NODES_CHECKLIST[n] in [None, False]]) > 0:
						self.change_nodes_status(WatcherState.NODES_ERROR)
					else:
						self.change_nodes_status(WatcherState.NODES_RUNNING)
				if self.nodes_status == WatcherState.NODES_RUNNING:
					self.change_robot_watcher(RobotState.ROBOT_READY)

			elif self.robot_watcher == RobotState.ROBOT_READY:
				if self.pin == WatcherState.PIN_OFF:
					self.change_robot_watcher(RobotState.ROBOT_RUNNING)

			# elif self.robot_watcher == RobotState.ROBOT_RUNNING:
			# 	pass

			# else: # self.robot_watcher == RobotState.ROBOT_HALT:
			# 	pass


			self.publish_nodes_status()

			r.sleep()


	def nodes_ready(self):
		for _, val in WatcherState.NODES_CHECKLIST.items():
			if val == None or val == False:
				return False
		return True


	def publish_robot_watcher(self):
		msg = RobotStatus(self.robot_watcher, self.nodes_status)
		self._robot_watcher_publisher.publish(msg)

	def publish_nodes_status(self):
		msg = NodesStatus()
		for node, val in WatcherState.NODES_CHECKLIST.items():
			if val == None:
				msg.nodes_init.append(node)
			elif val == True:
				msg.nodes_ready.append(node)
			else:
				msg.nodes_error.append(node)
		self._nodes_status_publisher.publish(msg)

	def change_robot_watcher(self, status):
		if status == RobotState.ROBOT_READY:
			rospy.loginfo("All node started: Robot standing by")
		elif status == RobotState.ROBOT_RUNNING:
			rospy.loginfo("Game started: Robot running")
		elif status == RobotState.ROBOT_HALT:
			rospy.loginfo("Game stoped: Robot halted")
		self.robot_watcher = status
		self.publish_robot_watcher()


	def change_nodes_status(self, status):
		if status == WatcherState.NODES_RUNNING:
			rospy.loginfo("All Nodes ready")
		elif status == WatcherState.NODES_ERROR :
			rospy.logerr("Nodes init not complete\nnodes not ready '{}'\nnodes unresponsive '{}"
				.format(
					str([n for n in WatcherState.NODES_CHECKLIST if WatcherState.NODES_CHECKLIST[n] == False]),
					str([n for n in WatcherState.NODES_CHECKLIST if WatcherState.NODES_CHECKLIST[n] == None])))
		self.nodes_status = status
		self.publish_robot_watcher()


	def set_readiness(self, msg):
		if msg.node_name in WatcherState.NODES_CHECKLIST:
			WatcherState.NODES_CHECKLIST[msg.node_name] = msg.ready
			return NodeReadinessResponse()
		else: 
			rospy.logwarn("Node {} not in cheklist".format(msg.node_name))
			return NodeReadinessResponse()

	def halt_game(self, event):
		# rospy.logwarn("!!!!!!!GAME STOP!!!!!!!!")
		self.change_robot_watcher(RobotState.ROBOT_HALT)

if __name__ == '__main__':
	RobotWatcherNode()
			

