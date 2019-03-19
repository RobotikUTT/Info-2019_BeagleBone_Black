#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in <mapping.xml>
from xml.etree import ElementTree
from importlib import import_module

import rospkg
import sys
import rospy
from rospy import Publisher, Subscriber

from interface_msgs import msg as interface_msgs
from can_msgs.msg import Frame

from can_interface.devices_handler import DevicesHandler

class CanInterfaceNode:
	def __init__(self):
		self.includes = {}
		self.subscribers = {}

		self.can_publisher = Publisher('/ros_can/sent_messages', Frame, queue_size=10)
		self.can_subscriber = Subscriber('/ros_can/received_messages', Frame, self.on_can_message)

		self.devices_handler = DevicesHandler(self)

	def include(self, message_name, package="interface_msgs"):
		# TODO handle std_msgs
		return getattr(interface_msgs, message_name)
	
	def subscribe(self, frame, subscriber):
		self.subscribers[frame] = subscriber

	def on_can_message(self, frame):
		if frame.id == Frame.BBB_CAN_ADDR:
			if frame.data[0] in self.subscribers:
				self.subscribers[frame.data[0]]
		# TODO handle WHOAMI frames


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