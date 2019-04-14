#!/usr/bin/python3
import rospy

from vision import ObjectsParser, MapParser

from node_watcher import Node

class VisionNode(Node):
	def __init__(self):
		obj_parser = ObjectsParser()
		map_parser = MapParser(obj_parser.objects)
		for obj in obj_parser.objects:
			print(obj_parser.objects[obj])
			print()
		print(map_parser.root)


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