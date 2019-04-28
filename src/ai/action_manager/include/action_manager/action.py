from .argumentable import Argumentable
from .object_requirement import ObjectRequirement
from .util import get_action_point_service

from typing import Union, List
import math

import rospy

from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, ActionStatus
from ai_msgs.srv import ComputeActionPoint

class ActionRepeater:
	pass

class Action:
	def __init__(self):
		self.name: str = ""
		self.points = 0
		self.repeat: Union[ActionRepeater, None] = None

		self.arguments = Argumentable()
		self.requirements: List[ObjectRequirement] = []
		self.parent: Union['ActionGroup', None] = None
	
		self.__action_point: Union[ActionPoint, None] = None
		self.__action_point_origin: Union[ActionPoint, None] = None

		self.__state = ActionStatus.IDLE
	
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
			
			try:
				self.__action_point = point_service(origin, self.arguments.to_list())
			except rospy.ServiceException:
				print("unable to process action point for {}".format(self.name))

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
			
	
	def __str__(self):
		return "[{}] points={}\n\t{}".format(
			self.name, self.total_points(), self.arguments.__str__()
		)