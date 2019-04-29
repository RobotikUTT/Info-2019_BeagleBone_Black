#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in <mapping.xml>

from xml.etree import ElementTree
from importlib import import_module

import threading, sys
import rospkg
import rospy
from rospy import Publisher, Subscriber

# Messages
from interface_msgs import msg as interface_msgs
from can_msgs.msg import Frame

# Devices handler
from can_interface import DevicesHandler

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

class CanInterfaceNode:
	def __init__(self):
		self.includes = {}
		self.subscribers = {}

		self.devices_handler = DevicesHandler(self)
		self.can_input_thread = threading.Thread(name="can_input", target=self.wait_for_can_message)
	
	def subscribe(self, frame, subscriber):
		self.subscribers[frame] = subscriber

	def wait_for_can_message(self):
		'''
			Loop in CAN bus to read data
		'''
		try:
			bus = can.interface.Bus(can_interface)
			for message in bus:
				self.on_can_message(message)

				if rospy.is_shutdown():
					return
		except:
			print("Can reception interrupted")
			bus.shutdown()


	def on_can_message(self, frame: can.Message):
		if frame.arbitration_id == Frame.BBB_CAN_ADDR or frame.arbitration_id == Frame.ALL_CAN_ADDR:
			if frame.data[0] in self.subscribers:
				self.subscribers[frame.data[0]]


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = CanInterfaceNode()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass