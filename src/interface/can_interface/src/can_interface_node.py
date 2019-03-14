#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in <mapping.xml>
import rospkg
import sys
import rospy

from xml.etree import ElementTree

from can_interface.io_elements import InputElement, OutputElement
from can_interface.interface import CanInterface

class CanInterfaceNode:
	def __init__(self):
		rospack = rospkg.RosPack()
		source_folder = rospack.get_path("can_interface")

		# Elements to be generated
		self.elements = []

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/mapping.xml").getroot()

		if root.tag != "mapping":
			print("invalid file, must contains a <mapping> root element")
			exit(0)

		self.interface = CanInterface()

		for child in root:
			if child.tag == "input":
				self.elements.append(InputElement(child.attrib, child, self.interface))
			elif child.tag == "output":
				self.elements.append(OutputElement(child.attrib, child, self.interface))
			else:
				print("unknow element :", child.tag)

		for el in self.elements:
			print(el)


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