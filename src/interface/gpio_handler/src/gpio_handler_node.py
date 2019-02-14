#!/usr/bin/python

import time
import rospy

from ai_msgs.msg import StartRobot, RobotStatus, Side

# Input pins for start signal and side
PIN_SIDE 	= "P8_7"
PIN_START 	= "P8_8"

RED_LED 	= "P9_23" # On while robot active
PINK_LED 	= "P9_25" # Blink while init, on while ready
GREEN_LED 	= "P9_27" # On while robot running 

GAME_IDLE = 0
GAME_STARTED = 1

# Leds displaying error code
ERROR_LED	= ["P9_" + str(j) for j in [17,18,12,15,30]]

# Groups of pins to ease initialisation
GPIO_IN_PINS = [ PIN_START, PIN_SIDE ]
GPIO_OUT_PINS = [ RED_LED, PINK_LED, GREEN_LED ] + ERROR_LED

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
		self.GPIO.output(RED_LED, self.GPIO.HIGH)

		# Subscribers
		rospy.Subscriber("/ai/controller/status", RobotStatus, self.setLED)

		# Publishers
		self._start_publisher = rospy.Publisher("/signal/start", StartRobot, queue_size = 1, latch=True)

		# init states
		self.side = -1 # impossible value that will get the side updated in the loop
		self.started = GAME_IDLE
		self.monitoring = True
		
	# Loop until ros terminate to monitor GPIO
	def loop(self):
		GPIO = self.GPIO

		# Monitor data
		r = rospy.Rate(5)
		while self.monitoring and not rospy.is_shutdown():
			# Update side if needed
			if GPIO.input(PIN_SIDE) != self.side:
				self.side = not self.side
			
			# Monitor start signal
			if GPIO.input(PIN_START) != GAME_IDLE and self.started == GAME_IDLE:
				self.started = not self.started
				self._start_publisher.publish(StartRobot(Side.LEFT if self.side else Side.RIGHT))

			# Wait for next cycle
			r.sleep()
			
		# Monitoring over : idling
		if not rospy.is_shutdown():
			rospy.spin()

		# After spinning, setting LEDs to low as it shutdown
		GPIO.output(RED_LED, GPIO.LOW)
		GPIO.output(PINK_LED, GPIO.LOW)
		GPIO.output(GREEN_LED, GPIO.LOW)

		# Leave GPIO guy if need
		simulated = rospy.get_param("/simulation", False)
		if simulated:
			from gpio_handler.GPIOemulator.EmulatorGUI import app
			app.callback()


	# Initialize GPIOs ports depending on whether it is a simulation or not
	def init_gpio(self):
		GPIO = None
		simulated = rospy.get_param("/simulation", False)
		
		# import GPIO depending whether it is a simulation
		if simulated:
			from gpio_handler.GPIOemulator.EmulatorGUI import GPIO
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


	# set the status led on the right combination
	def setLED(self, msg):
		robot_status = msg.robot_status
		error_code = msg.error_code
		GPIO = self.GPIO

		# if the robot is running, we stop monitoring pin inputs
		if robot_status == RobotStatus.ROBOT_RUNNING:
			self.monitoring = False

		# error case
		if error_code != 0:
			GPIO.output(PINK_LED, GPIO.LOW)
			GPIO.output(GREEN_LED, GPIO.LOW)
			for i in xrange(5):
				GPIO.output(ERROR_LED[i], error_code >> i & 1)

		else:
			# Pink led blink on init or is on when ready
			GPIO.output(PINK_LED,
				(robot_status == RobotStatus.ROBOT_INIT and not GPIO.input(PINK_LED)) or
				robot_status == RobotStatus.ROBOT_READY
			)

			# Green led active when the robot is running
			GPIO.output(GREEN_LED,
				robot_status == RobotStatus.ROBOT_RUNNING
			)

if __name__ == '__main__':
	handler = GPIOHandlerNode()
	handler.loop()
