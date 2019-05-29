#!/usr/bin/python

import time
import rospy

import Adafruit_BBIO.GPIO as GPIO

from ai_msgs.msg import StartRobot, RobotStatus, Side

# Input pins for start signal and side
PIN_SIDE 	= "P8_7"
PIN_START 	= "P8_8"

GAME_IDLE = 0
GAME_STARTED = 1

# Groups of pins to ease initialisation
GPIO_IN_PINS = [ PIN_START, PIN_SIDE ]

# @brief I/O interface between the robot pins and the IA
class GPIOHandlerNode(object):

	##
	## @brief	  Constructs the object.
	##
	## @param	  self  The object
	##
	def __init__(self):
		rospy.init_node("robot_gpio_handler", log_level=rospy.INFO)

		# init GPIOs
		self.GPIO = self.init_gpio()

		# Publishers
		self._start_publisher = rospy.Publisher("/signal/start", StartRobot, queue_size = 1, latch=True)

		# init states
		self.side = -1 # impossible value that will get the side updated in the loop
		self.started = GAME_IDLE
		
	# Loop until ros terminate to monitor GPIO
	def loop(self):

		# Monitor data
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			# Update side if needed
			if GPIO.input(PIN_SIDE) != self.side:
				self.side = not self.side
			
			# Monitor start signal
			if GPIO.input(PIN_START) != GAME_IDLE and self.started == GAME_IDLE:
				self.started = not self.started
				self._start_publisher.publish(StartRobot(Side.UP if self.side else Side.DOWN))

			# Wait for next cycle
			r.sleep()


	# Initialize GPIOs ports
	def init_gpio(self):
		# set pins
		for pin in GPIO_IN_PINS:
			GPIO.setup(pin, GPIO.IN)

		return GPIO

if __name__ == '__main__':
	handler = GPIOHandlerNode()
	handler.loop()
