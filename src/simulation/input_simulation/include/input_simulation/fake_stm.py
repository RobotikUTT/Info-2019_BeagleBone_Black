import rospy

from tf.transformations import euler_from_quaternion

from interface_msgs.msg import Point, Speed, StmMode, RobotBlocked, StmDone
from interface_description.msg import InterfaceTopics as Topics
from ai_msgs.msg import NodeStatus

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import Pose
from node_watcher.node import Node

from typing import List

# Consts for goals
ORIENTED_POSITION = 2
POSITION = 1
ORIENTATION = 0

class FakeStm(Node):
	def __init__(self):
		super().__init__("STM", "board")

		self.goals = []
		self.pose = Point()
		self.pose_offset = Point()
		self.mode: int = StmMode.STOP
		self.goal = None

		# Gazebo services
		self.position_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

		# Publishers
		self.pose_pub = rospy.Publisher(Topics.STM_POSITION, Point)
		self.speed_pub = rospy.Publisher(Topics.STM_SPEED, Speed)

		self.finished_pub = rospy.Publisher(Topics.ALL_ORDER_DONE, StmDone)
		self.robot_blocked_pub = rospy.Publisher(Topics.STM_ROBOT_BLOCKED, RobotBlocked)

		# Subscribers
		self.speed_setter_sub = rospy.Subscriber(Topics.STM_SET_SPEED, Speed, self.set_speed)
		self.set_mode_sub = rospy.Subscriber(Topics.STM_SET_MODE, StmMode, self.set_mode)

		# Basic orders
		self.go_to_angle_sub = rospy.Subscriber(Topics.STM_GO_TO_ANGLE, Point, self.go_to_angle)
		self.go_to_sub = rospy.Subscriber(Topics.STM_GO_TO, Point, self.go_to)
		self.rotate_sub = rospy.Subscriber(Topics.STM_ROTATE, Point, self.rotate)

		self.pose_setter_sub = rospy.Subscriber(Topics.STM_SET_POSE, Point, self.set_pose)


		self.set_node_status(NodeStatus.READY)

	def set_speed(self, msg):
		# TODO
		pass

	def set_mode(self, msg):
		self.mode = msg.mode

	def go_to_angle(self, msg: Point):
		self.add_goal(msg, ORIENTED_POSITION)

	def go_to(self, msg: Point):
		self.add_goal(msg, POSITION)

	def rotate(self, msg: Point):
		self.add_goal(msg, ORIENTATION)

	def set_pose(self, msg):
		# Update offset for gazebo messages
		self.pose_offset.x = self.pose_offset.x + self.pose.x - msg.x
		self.pose_offset.y = self.pose_offset.y + self.pose.y - msg.y
		self.pose_offset.angle = self.pose_offset.angle + (self.pose.angle - msg.angle)
		
		# Set position
		self.pose = msg

	def add_goal(self, point: Point, type: int) -> None:
		self.goals.append((type, point))
	
	def get_z_angle(self, pose: Pose):
		x, y, z = euler_from_quaternion(
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w
		)

		return z

	def spinOnce(self):
		request = GetModelStateRequest()
		request.model_name = "beagle_gazebo"

		response = self.position_service(request)

		# Update and publish position
		self.pose.x = response.pose.position.x + self.pose_offset.x
		self.pose.y = response.pose.position.y + self.pose_offset.y
		self.pose.angle = self.get_z_angle(response.pose) + self.pose_offset.angle
		self.pose_pub.publish(self.pose)

def register(master_node):
	'''Function defining a simulation extentions'''
	print("registering STM")
	stm = FakeStm()
	master_node.spin_callbacks.append(stm.spinOnce)