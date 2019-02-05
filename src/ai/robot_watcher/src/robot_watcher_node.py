#!/usr/bin/python

## @file robot_watcher_node.py
##    @brief Node class wich monitor all nodes of the project.
##    
##    
##    @author Alexis CARE
##

import time
import rospy

from ai_msgs.msg import NodesStatus, RobotStatus, SetSide
from ai_msgs.srv import NodeReadiness, NodeReadinessResponse

from robot_watcher.RStatus.State import RobotState, WatcherState, NODES_CHECKLIST, Side

from std_msgs.msg import Empty

## 
## @defgroup Robot_watcher The Robot_watcher package
## @{

PIN_SIDE 	= "P8_7"
PIN_START 	= "P8_8"

RED_LED 	= "P9_23"
PINK_LED 	= "P9_25"
GREEN_LED 	= "P9_27"

ERROR_LED	= ["P9_" + str(j) for j in [17,18,12,15,30]]

GPIO_IN_PINS = [ PIN_START, PIN_SIDE ]
GPIO_OUT_PINS = [ RED_LED, PINK_LED, GREEN_LED ] + ERROR_LED

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

		# init GPIOs
		GPIO = self.init_gpio()
		GPIO.output(RED_LED, GPIO.HIGH)

		# init node readiness service
		rospy.Service("/ai/robot_watcher/node_readiness", NodeReadiness, self.set_readiness)
		
		# init publishers
		self._robot_watcher_publisher 	= rospy.Publisher("/ai/robot_watcher/robot_status", RobotStatus, queue_size = 1, latch=True)
		self._nodes_status_publisher 	= rospy.Publisher("/ai/robot_watcher/nodes_status", NodesStatus, queue_size = 1)
		
		self._side_publisher 			= rospy.Publisher("side", SetSide, queue_size = 1, latch=True)
		
		self._ping_publisher 			= rospy.Publisher("/ALL/Ping", Empty, queue_size = 3)

		# init states
		self.robot_state 	= RobotState.ROBOT_INIT
		self.nodes_status 	= WatcherState.NODES_INIT
		self.pin 			= WatcherState.PIN_ON

		self.init_start_time = time.time()

		self.publish_robot_status()

		self.side = GPIO.input(PIN_SIDE)
		self._side_publisher.publish(SetSide(self.side))

		self.error_code = 0

		self.ping_board = rospy.Timer(rospy.Duration(1), self.board_ping)

		r = rospy.Rate(5)
		while not rospy.is_shutdown():

			if self.pin == WatcherState.PIN_ON:
				if GPIO.input(PIN_SIDE) != self.side:
					self.side = not self.side
					rospy.logwarn("side: {}".format(self.side))
					self._side_publisher.publish(SetSide(self.side))

				if GPIO.input(PIN_START) == WatcherState.PIN_OFF:
					self.pin = WatcherState.PIN_OFF
					rospy.Timer(rospy.Duration(self.GAME_LENTH), self.halt_game, oneshot=True)


			if self.robot_state == RobotState.ROBOT_INIT:
				if self.nodes_ready():
					self.change_nodes_status(WatcherState.NODES_RUNNING)
				if time.time() - self.init_start_time > self.INIT_TIMEOUT:
					if len([n for n in NODES_CHECKLIST if NODES_CHECKLIST[n] in ["need", False]]) > 0:
						self.change_nodes_status(WatcherState.NODES_ERROR)
					else:
						self.change_nodes_status(WatcherState.NODES_RUNNING)
				if self.nodes_status == WatcherState.NODES_RUNNING:
					self.set_robot_status(RobotState.ROBOT_READY)

			elif self.robot_state == RobotState.ROBOT_READY:
				if self.pin == WatcherState.PIN_OFF:
					self.set_robot_status(RobotState.ROBOT_RUNNING)

			# elif self.robot_state == RobotState.ROBOT_RUNNING:
			# 	pass

			# else: # self.robot_state == RobotState.ROBOT_HALT:
			# 	pass


			self.publish_nodes_status()
			self.set_LED(GPIO)
			r.sleep()

		GPIO.output(RED_LED, 	GPIO.LOW)
		GPIO.output(PINK_LED,	GPIO.LOW)
		GPIO.output(GREEN_LED, 	GPIO.LOW)

	def init_gpio(self):
		GPIO = None
		simulated = rospy.get_param("/simulation", False)
		
		# import GPIO depending wether it is a simulation
		if simulated:
			from robot_watcher.GPIOemulator.EmulatorGUI import GPIO
			GPIO.setmode(GPIO.BCM)
			GPIO.setwarnings(False)
		else:
			import Adafruit_BBIO.GPIO as GPIO
		
		# set pins
		for pin in GPIO_IN_PINS:
			if simulated:
				GPIO.setup(pin, GPIO.IN, initial = GPIO.LOW)
			else:
				GPIO.setup(pin, GPIO.IN)

		for pin in GPIO_OUT_PINS:
			if simulated:
				GPIO.setup(pin, GPIO.OUT, initial = GPIO.LOW)
			else:
				GPIO.setup(pin, GPIO.OUT)

		return GPIO



	##
	## @brief      Set the status led on the right combination.
	##
	## @param      self  The object
	## @param      GPIO  The gpio output
	##
	##
	def set_LED(self, GPIO):
		# error case
		if self.error_code != 0:
			GPIO.output(PINK_LED, GPIO.LOW)
			GPIO.output(GREEN_LED, GPIO.LOW)
			for i in xrange(5):
				GPIO.output(ERROR_LED[i], self.error_code >> i & 1)

		else:
			# Pink led blink on init or is on when ready
			GPIO.output(PINK_LED,
				(self.robot_state == RobotState.ROBOT_INIT and not GPIO.input(PINK_LED)) or
				self.robot_state == RobotState.ROBOT_READY
			)

			# Green led active when the robot is running
			GPIO.output(GREEN_LED,
				self.robot_state == RobotState.ROBOT_RUNNING
			)

			'''if self.robot_state == RobotState.ROBOT_INIT:
				GPIO.output(PINK_LED, not GPIO.input(PINK_LED))
			elif self.robot_state == RobotState.ROBOT_READY:
				GPIO.output(PINK_LED, GPIO.HIGH)
			elif self.robot_state == RobotState.ROBOT_RUNNING:
				GPIO.output(GREEN_LED, GPIO.HIGH)'''

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
		msg = RobotStatus(self.robot_state, self.nodes_status)
		self._robot_watcher_publisher.publish(msg)

	##
	## @brief      Publish nodes status
	##
	## @param      self  The object
	##
	def publish_nodes_status(self):
		msg = NodesStatus()
		for node, val in NODES_CHECKLIST.items():
			if val == "need" or val == "optional":
				msg.nodes_init.append(node)
			elif val == True:
				msg.nodes_ready.append(node)
			else:
				msg.nodes_error.append(node)

		self._nodes_status_publisher.publish(msg)

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
			NODES_CHECKLIST[msg.node_name] = msg.ready
			self.error_code = msg.error_code
			return NodeReadinessResponse()
		else:
			rospy.logwarn("Node {} not in cheklist".format(msg.node_name))
			return NodeReadinessResponse()

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
