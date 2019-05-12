#!/usr/bin/python3
import rospy

from vision import RectZone, Offset, Zone

from node_watcher import Node

from visualization_msgs.msg import Marker
from ai_msgs.srv import DeclareZone
from ai_msgs.msg import Shape, NodeStatus

class VisionNode(Node):
	# TODO map publishing
	# TODO objet
	def __init__(self):
		super().__init__("vision", "ai")

		self.zone = RectZone.parse_file("/map/table.xml", package="ai_description")

		self.rviz_marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 100)

		rospy.loginfo("map parsed, waiting for pathfinding service to register zones")
		self.declare_zone = rospy.ServiceProxy("/ai/vision/blocking_zone", DeclareZone)
		
		try:
			self.declare_zone.wait_for_service(1)
		except rospy.exceptions.ROSException:
			rospy.logerr("unable to reach pathfinding service")
			self.set_status(NodeStatus.ERROR)
			return

		rospy.loginfo("pathfinding service joined, starting data input")

		# Initial publication of all zone
		for zone in self.zone.zones:
			# Create request
			shape = Shape()
			shape.x = zone.x
			shape.y = zone.y

			if isinstance(shape, RectZone):
				shape.params = [ shape.width, shape.height ]
			elif isinstance(shape, CircleShape):
				shape.params = [ shape.radius ]
			else:
				raise Exception("shape type not handled : {}".format(zone))

			# Publish it to pathfinder
			self.declare_zone(shape, False)

		# We are ready !
		rospy.loginfo("pathfinding initialized")
		self.set_status(NodeStatus.READY)

	def get_rviz_marker(self, zone: Zone):
		# Create marker and set it up
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.id = zone.id
		marker.action = marker.ADD

		# Set marker position and orientation
		marker.pose.position.x = zone.x
		marker.pose.position.y = zone.y
		marker.pose.position.z = 0
		marker.pose.orientation.w = 1

		if isinstance(zone, RectZone):
			marker.type = marker.CYLINDER
			marker.scale.x = zone.width / 100
			marker.scale.y = zone.height / 100
			marker.scale.z = 2
		elif isinstance(shape, CircleShape):
			marker.type = marker.SPHERE
			marker.scale.x = zone.width / 100
			marker.scale.y = zone.height / 100
			marker.scale.z = 2
		
		return marker

	def update_rviz(self):
		# Publish ground
		ground = self.get_rviz_marker(self.zone)
		ground.scale.z = 1
		self.rviz_marker_pub.publish(ground)


		# Publish all zones
		for zone in self.zone.zones:
			self.rviz_marker_pub.publish(self.get_rviz_marker(zone))

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = VisionNode()

		# Spin
		while not rospy.is_shutdown():
			#node.update_rviz()
			rospy.spin()
	except rospy.ROSInterruptException:
		pass