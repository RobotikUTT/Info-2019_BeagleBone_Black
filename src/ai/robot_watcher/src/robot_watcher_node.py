#!/usr/bin/python

## @file robot_watcher_node.py
##    @brief Node class wich monitor all nodes of the project.
##    
##    
##    @author Alexis CARE
##

import time
import rospy

from ai_msgs.msg import RobotStatus, SetSide, NodeStatusUpdate
from ai_msgs.srv import NodeReadiness, NodeReadinessResponse

from robot_watcher.RStatus.State import RobotState, WatcherState, NODES_CHECKLIST, Side

from std_msgs.msg import Empty


WAIT_TIME = 10

##
## @brief      Class for robot watcher node.
## 
## @details    This node monitors all the nodes of the project and is the interface for the human
##
class RobotWatcherNode(object):

	##
	## @brief      Constructs the object.
	##
	## @param      self  The object
	##
	def __init__(self):
		rospy.init_node("ai_robot_watcher", log_level=rospy.INFO)

		self.INIT_TIMEOUT = rospy.get_param("~INIT_TIMEOUT", 15)
		self.GAME_LENTH = rospy.get_param("~GAME_LENTH", 10)


		# init node readiness service
		rospy.Subscriber("/ai/node_watcher/update", NodeStatusUpdate, self.set_readiness)
		rospy.Subscriber("start", Empty, self.set_started)

		# init publishers
		self._robot_status_publisher  = rospy.Publisher("/ai/robot_watcher/robot_status", RobotStatus, queue_size = 1, latch=True)
		self._ping_publisher = rospy.Publisher("/ALL/Ping", Empty, queue_size = 3)

		# init states
		self.robot_state = RobotState.ROBOT_INIT
		self.nodes_status = WatcherState.NODES_INIT
		self.start_signal_received = False # if received start from GPIO
		self.init_start_time = time.time()
		self.error_code = 0

		# First publish robot status
		self.publish_robot_status()
	
		# Init ping on board
		self.ping_board = rospy.Timer(rospy.Duration(1), self.board_ping)

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			# in case of init
			if self.robot_state == RobotState.ROBOT_INIT:
				# watch for nodes status
				if self.nodes_ready():
					self.change_nodes_status(WatcherState.NODES_RUNNING)

				if time.time() - self.init_start_time > self.INIT_TIMEOUT:
					if len([n for n in NODES_CHECKLIST if NODES_CHECKLIST[n] in ["need", False]]) > 0:
						self.change_nodes_status(WatcherState.NODES_ERROR)
					else:
						self.change_nodes_status(WatcherState.NODES_RUNNING)

				# if nodes are ready we can start the robot
				if self.nodes_status == WatcherState.NODES_RUNNING:
					self.set_robot_status(RobotState.ROBOT_READY)

			elif self.robot_state == RobotState.ROBOT_READY:
				# if the robot is ready and we received start signal
				if self.start_signal_received:
					self.set_robot_status(RobotState.ROBOT_RUNNING)

			r.sleep()



	def set_started(self, msg):
		self.start_signal_received = True

	##
	## @brief      Test if all needed nodes are ready
	##
	## @param      self  The object
	##
	## @return     All nodes ready
	##
	def nodes_ready(self):
		for _, val in NODES_CHECKLIST.items():
			if val == "need" or val == False:
				return False
		return True

	##
	## @brief      Publish the robot status
	##
	## @param      self  The object
	##
	def publish_robot_status(self):
		msg = RobotStatus(self.robot_state, self.nodes_status, self.error_code)
		self._robot_status_publisher.publish(msg)


	##
	## @brief      Change the robot status
	##
	## @param      self    The object
	## @param      status  The status
	##
	def set_robot_status(self, status):
		if status == RobotState.ROBOT_READY:
			rospy.loginfo("All node started: Robot standing by")
		elif status == RobotState.ROBOT_RUNNING:
			rospy.loginfo("Game started: Robot running")
		elif status == RobotState.ROBOT_HALT:
			rospy.loginfo("Game stoped: Robot halted")
		self.robot_state = status
		self.publish_robot_status()

	##
	## @brief      Change a node status
	##
	## @param      self    The object
	## @param      status  The status
	##
	def change_nodes_status(self, status):
		if status == WatcherState.NODES_RUNNING:
			rospy.loginfo("All nodes are ready")
		elif status == WatcherState.NODES_ERROR :
			rospy.logerr("Nodes init not complete\nnodes not ready '{}'\nnodes unresponsive '{}"
				.format(
					str([n for n in NODES_CHECKLIST if NODES_CHECKLIST[n] == False]),
					str([n for n in NODES_CHECKLIST if NODES_CHECKLIST[n] == "need"])))
		
		self.nodes_status = status
		self.publish_robot_status()


	##
	## @brief      Service to set a node readiness
	##
	## @param      self  The object
	## @param      msg   The service message
	##
	def set_readiness(self, msg):
		if msg.node_name in NODES_CHECKLIST:
			NODES_CHECKLIST[msg.node_name] = msg.status
			self.error_code = msg.error_code
		else:
			rospy.logwarn("Node {} not in cheklist".format(msg.node_name))

	##
	## @brief      Stop the robot at the end of the game
	##
	## @param      self   The object
	## @param      event  The end of timer event
	##
	##
	def halt_game(self, event):
		# rospy.logwarn("!!!!!!!GAME STOP!!!!!!!!")
		self.set_robot_status(RobotState.ROBOT_HALT)
		# rospy.Timer(rospy.Duration(WAIT_TIME), self.shutdown_ros, oneshot=True)

	##
	## @brief      Shutdown ros
	##
	## @details    Not used
	##
	## @param      self   The object
	## @param      event  The end of timer event
	## 
	## @todo Include this methode into the project
	##
	def shutdown_ros(self, event):
		rospy.signal_shutdown("shuting down: end of game")

	##
	## @brief      Chek if the LED board is connected
	##
	## @param      self  The object
	##
	## @return     LED board connected
	## 
	##
	def board_ready(self):
		for name, val in NODES_CHECKLIST.items():
			if name.split('/')[1] == "board":
				if val == "need" or val == False:
					return False
		return True

	##
	## @brief      Ping the board on the network
	##
	## @param      self   The object
	## @param      event  The end of timer event
	## 
	##
	def board_ping(self, event):
		if NODES_CHECKLIST["/ros_can/interface"] != False or NODES_CHECKLIST["/ros_can/interface"] != "need" :
			if not self.board_ready():
				self._ping_publisher.publish(Empty())
			else:
				self.ping_board.shutdown()


##
## @}
##
if __name__ == '__main__':
	RobotWatcherNode()
