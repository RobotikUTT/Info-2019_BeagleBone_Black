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
from can_interface.io_elements import InputElement, OutputElement

class CanInterfaceNode:
	def __init__(self):
		self.includes = {}
		self.subscribers = {}

		self.can_publisher = Publisher('sent_messages', Frame, queue_size=10)
		self.can_subscriber = Subscriber('received_messages', Frame, self.on_can_message)

		self.devices_handler = DevicesHandler(self)

		# Elements to be generated
		self.elements = []

		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("interface_description")

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/can/mapping.xml").getroot()

		if root.tag != "mapping":
			print("invalid file, must contains a <mapping> root element")
			exit(0)

		for child in root:
			if child.tag == "input":
				self.elements.append(InputElement(child.attrib, child, self))
			elif child.tag == "output":
				self.elements.append(OutputElement(child.attrib, child, self))
			else:
				print("unknow element :", child.tag)

		for el in self.elements:
			rospy.logdebug(el)
	
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