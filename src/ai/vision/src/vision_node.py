#!/usr/bin/python3
import rospy

from vision import ObjectsParser, MapParser

from node_watcher import Node

class VisionNode(Node):
	def __init__(self):
		pass


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = VisionNode()

		# Spin
		while not rospy.is_shutdown():
			#rospy.spin()
			break
	except rospy.ROSInterruptException:
		pass