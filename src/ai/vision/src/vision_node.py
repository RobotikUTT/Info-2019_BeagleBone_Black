#!/usr/bin/python3
import rospy

from vision import ObjectsParser

from node_watcher import Node

class VisionNode(Node):
	def __init__(self):
		parser = ObjectsParser()
		print(parser.objects)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = VisionNode()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass