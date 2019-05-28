from args_lib.argumentable import Argumentable
from .util import get_action_point_service

from typing import Union, List, Type
import math

import rospy

from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, ActionStatus
from args_lib.msg import Argument
from ai_msgs.srv import ComputeActionPoint

from xml_class_parser import Parsable, Bind, BindDict, BindList
from xml_class_parser.types import Bool, BlackList

class ActionChoice:
	"""
		Choice of an action
	"""
	def __init__(self, action=None, score = 0):
		self.action = action
		self.score = score

Argument = Parsable(name="arg", attributes={"name": str}, content=Bind(to="value"))(Argument)


@Parsable(
	name = Bind(to="name", type=BlackList("group")),
	attributes = {
		"native": Bool,
		"points": int,
	},
	children = [
		BindList(to="arguments", type=Argument)
	]
)
class Action:
	action_id = 0

	def __init__(self):
		# Object id
		self.action_id = Action.action_id
		Action.action_id += 1

		self.name: str = ""
		self.native = False
		self.points = 0

		self.arguments: Argumentable = []
		self.requirements: List[ObjectRequirement] = []
		self.parent: Union['ActionGroup', None] = None
	
		self.__action_point: Union[ActionPoint, None] = None
		self.__action_point_origin: Union[ActionPoint, None] = None

		self.__state = ActionStatus.IDLE

	def set_side(self, side: int):
		# Set side as argument
		self.arguments.set("_side", side)

	def __before_children__(self, context):
		# Get parent arguments
		if context.parent is not None:
			self.arguments.extend(context.parent.arguments)

	def __parsed__(self, context):
		self.arguments = Argumentable().from_list(self.arguments)

		# Apply context to values
		self.points = self.__contextualized_value(str(self.points), int)

		for key in self.arguments.keys():
			self.arguments.set(key, self.__contextualized_value(self.arguments.get(key)))
		
	def __contextualized_value(self, value: str, cast: Type = str):
		# TODO check
		if len(value) > 0 and value[0] == "@":
			if self.arguments.has(value[1:]):
				return cast(self.arguments.get(value[1:]))
			else:
				return cast()
		else:
			return cast(value) if len(value) > 0 else cast()

	@property
	def state(self) -> int:
		return self.__state

	@state.setter
	def state(self, state: int):
		self.__state = state

		# If not resuming, we propagate to parent
		if state != ActionStatus.IDLE and self.parent != None:
			self.parent.state = state


	def total_points(self):
		return self.points

	
	def action_point(self, origin: Pose2D) -> ActionPoint:
		'''
			Compute the initial and final point of the action.
			Calls associated performer service to let it compute the point based on args 
		'''

		# Check if previously saved for this origin
		if self.__action_point == None or self.__action_point_origin != origin:
			point_service = rospy.ServiceProxy(
				get_action_point_service(self.name),
				ComputeActionPoint
			)
			
			# Wait 2s max for service
			point_service.wait_for_service(2)

			try:
				self.__action_point = point_service(origin, self.arguments.to_list()).action_point
				self.__action_point_origin = origin
			except rospy.ServiceException:
				rospy.logerr("unable to process action point for {}".format(self.name))

		return self.__action_point

	def travel_distance(self, origin: Pose2D) -> float:
		'''Compute the estimated distance to travel before the robot reach the end of the action'''
		point = self.action_point(origin)

		# Return euclidian distance
		return math.sqrt(
			pow(point.start.x - origin.x, 2) +
			pow(point.start.y - origin.y, 2)
		) + math.sqrt(
			pow(point.start.x - point.end.x, 2) +
			pow(point.start.y - point.end.y, 2)
		)


	def priority(self, origin: Pose2D) -> float:
		points = self.total_points()
		distance = self.travel_distance(origin)

		if distance == 0: # handle divide by 0
			return math.inf
		else:
			return points * points / distance
			
	def get_optimal(self, robot_pos: Pose2D) -> ActionChoice:
		"""
			Returns the optimal choice of native action from this action
		"""

		# In a simple action, only two cases : action unavailable or not
		if self.state != ActionStatus.IDLE:
			return ActionChoice()
		else:
			return ActionChoice(self, self.priority(robot_pos))

	def color(self, text):
		if self.state == ActionStatus.PAUSED:
			# Yellow
			return '\033[33m' + text + '\033[0m'
		elif self.state == ActionStatus.DONE:
			# Green
			return '\033[32m' + text + '\033[0m'
		elif self.state == ActionStatus.IDLE:
			# Grey
			return '\033[90m' + text + '\033[0m'
		else:
			# Red
			return '\033[31m' + text + '\033[0m'

	def __str__(self):
		return "{} points={} {{{}}}".format(
			self.color("[{}@{}]".format(self.name, self.action_id)), self.total_points(), self.arguments.__str__()
#			self.color("[{}@{}]".format(self.name, self.action_id)), self.total_points(), ""
		)