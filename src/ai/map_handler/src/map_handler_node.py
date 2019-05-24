#!/usr/bin/python3
import rospy
import math

from map_handler import RectZone, Offset, Zone, CircleZone

from node_watcher import Node

from visualization_msgs.msg import Marker
from ai_msgs.msg import Shape, NodeStatus, Side
from ai_msgs.srv import DeclareZone, GetMapSize, GetMapSizeResponse, \
	GetSidedPoint, GetSidedPointResponse

from pathfinder.srv import SetMapDimension, SetRobotRadius

class MapHandlerNode(Node):
	def __init__(self):
		super().__init__("map_handler", "ai")

		# Get root zone
		self.zone = RectZone.parse_file("/map/table.xml", package="ai_description")
		self.map_size_srv = rospy.Service("/ai/map_handler/get_map_size", GetMapSize, self.get_map_size)
		self.size_point_srv = rospy.Service("/ai/map_handler/get_sided_point", GetSidedPoint, self.get_sided_point)

		# RViz publisher
		self.rviz_marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 100)

		if self.pathfinder_update():
			# We are ready !
			self.set_status(NodeStatus.READY)

	def get_sided_point(self, req):
		"""
			Handle get_sided_point service
			Takes a point from the original (DOWN) side and get equivalent
			to given side (UP/DOWN)
		"""
		res = GetSidedPointResponse()
		res.point = req.point

		if req.side == Side.UP:
			res.point.y = self.zone.height - res.point.y
			res.point.theta = math.pi * 1000 - res.point.theta # half a turn in mrad

		return res

	def get_map_size(self, req):
		res = GetMapSizeResponse()
		res.width = self.zone.width
		res.height = self.zone.height
		return res


	def pathfinder_update(self) -> bool:
		# Pathfinder
		self.declare_zone = rospy.ServiceProxy("/ai/pathfinder/declare_zone", DeclareZone)
		self.set_map_dim = rospy.ServiceProxy("/ai/pathfinder/set_map_dimension", SetMapDimension)
		self.set_robot_radius = rospy.ServiceProxy("/ai/pathfinder/set_robot_radius", SetRobotRadius)

		try:
			self.set_map_dim.wait_for_service(1)
			self.declare_zone.wait_for_service(1)
			self.set_robot_radius.wait_for_service(1)
		except rospy.exceptions.ROSException as e:
			raise e
			rospy.logerr("unable to reach pathfinding service")
			self.set_status(NodeStatus.ERROR)
			return False

		# Reset remote map
		self.set_map_dim(self.zone.width, self.zone.height) # TODO make pathfinder use get_map_size
		self.set_robot_radius(22)

		# Initial publication of all zone
		zones = self.zone.zones
		while len(zones) > 0:
			zone = zones.pop()

			if zone.blocking:
				# Create request
				shape = Shape()
				shape.x = zone.x
				shape.y = zone.y

				if isinstance(zone, RectZone):
					shape.type = Shape.RECT
					shape.params = [ zone.width, zone.height ]
				elif isinstance(zone, CircleZone):
					shape.type = Shape.CIRCLE
					shape.params = [ zone.radius ]
				else:
					raise Exception("shape type not handled : {}".format(type(zone)))

				# Publish it to pathfinder
				self.declare_zone(shape, False)
			else:
				# Append children
				zones.extend(zone.zones)

		
		return True

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
	rospy.init_node('map_handler')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = MapHandlerNode()

		# Spin
		while not rospy.is_shutdown():
			#node.update_rviz()
			rospy.spin()
	except rospy.ROSInterruptException:
		pass