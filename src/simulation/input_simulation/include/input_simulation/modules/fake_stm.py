import rospy
import math

from interface_msgs.msg import DirectedPose, Speed, StmMode, RobotBlocked, StmDone
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
		super().__init__("stm", "board")

		self.goals = []
		self.pose = DirectedPose()
		self.pose_offset = DirectedPose()
		self.mode: int = StmMode.STOP
		self.goal = None

		# Gazebo services
		self.position_service = rospy.ServiceProxy("/simulation/gazebo/get_model_state", GetModelState)

		# Publishers
		self.pose_pub = rospy.Publisher(Topics.STM_POSITION, DirectedPose, queue_size=10)
		self.speed_pub = rospy.Publisher(Topics.STM_SPEED, Speed, queue_size=10)

		self.finished_pub = rospy.Publisher(Topics.ALL_ORDER_DONE, StmDone, queue_size=10)
		self.robot_blocked_pub = rospy.Publisher(Topics.STM_ROBOT_BLOCKED, RobotBlocked, queue_size=10)

		# Subscribers
		self.speed_setter_sub = rospy.Subscriber(Topics.STM_SET_SPEED, Speed, self.set_speed)
		self.set_mode_sub = rospy.Subscriber(Topics.STM_SET_MODE, StmMode, self.set_mode)

		# Basic orders
		self.go_to_angle_sub = rospy.Subscriber(Topics.STM_GO_TO_ANGLE, DirectedPose, self.go_to_angle)
		self.go_to_sub = rospy.Subscriber(Topics.STM_GO_TO, DirectedPose, self.go_to)
		self.thetaate_sub = rospy.Subscriber(Topics.STM_ROTATE, DirectedPose, self.thetaate)

		self.pose_setter_sub = rospy.Subscriber(Topics.STM_SET_POSE, DirectedPose, self.set_pose)


		self.set_status(NodeStatus.READY)

	def set_speed(self, msg):
		# TODO
		pass

	def set_mode(self, msg):
		self.mode = msg.mode

	def go_to_angle(self, msg: DirectedPose):
		self.add_goal(msg, ORIENTED_POSITION)

	def go_to(self, msg: DirectedPose):
		self.add_goal(msg, POSITION)

	def rotate(self, msg: DirectedPose):
		self.add_goal(msg, ORIENTATION)

	def set_pose(self, msg):
		# Update offset for gazebo messages
		self.pose_offset.x = self.pose_offset.x + self.pose.x - msg.x
		self.pose_offset.y = self.pose_offset.y + self.pose.y - msg.y
		self.pose_offset.theta = self.pose_offset.theta + (self.pose.theta - msg.theta)
		
		# Set position
		self.pose = msg

	def add_goal(self, point: DirectedPose, type: int) -> None:
		self.goals.append((type, point))
	
	def get_z_angle(self, pose: Pose):
		# From [http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/]	
		z = pose.orientation.z
		w = pose.orientation.w
		
		s = math.sqrt(1 - w * w) # assuming quaternion normalised then w is less than 1, so term always positive.
		
		if s < 0.001: # test to avoid divide by zero, s is always positive due to sqrt
			# if s close to zero then direction of axis not important
			return 1
		else:
			return z / s

	def spinOnce(self):
		request = GetModelStateRequest()
		request.model_name = "beagle_gazebo"

		response = self.position_service(request)

		# Update and publish position
		self.pose.x = response.pose.position.x + self.pose_offset.x
		self.pose.y = response.pose.position.x + self.pose_offset.y
		self.pose.theta = self.get_z_angle(response.pose) + self.pose_offset.theta
		self.pose_pub.publish(self.pose)

def register(master_node):
	'''Function defining a simulation extentions'''
	stm = FakeStm()
	master_node.spin_callbacks.append(stm.spinOnce)