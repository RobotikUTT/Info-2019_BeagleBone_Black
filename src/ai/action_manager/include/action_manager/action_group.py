from .action import Action
from .action_repeat import ActionRepeater
from .object_requirement import ObjectRequirement

from ai_msgs.msg import ActionPoint, ActionStatus
from geometry_msgs.msg import Pose2D

from typing import List, Union

from xml_class_parser import Parsable, Bind, BindDict, Enum, BindList

@Parsable(
	name = "group",
	attributes = {
		"type": Enum(binding = {"ordered": 0, "unordered": 1, "best": 2}),
		"repeat": ActionRepeater
	},
	children = [
		BindList(to="children", type=Action),
		BindList(to="none", type=ObjectRequirement),
		BindList(to="children", type=Parsable.SELF)
	]
)
class ActionGroup(Action):
	ORDERED = 0 # actions done in order
	UNORDERED = 1 # actions done in any order
	BEST = 2 # only the best action is made

	def __init__(self):
		super().__init__()

		self.type: int = ActionGroup.ORDERED
		self.children: List[Action] = []
	
		self.__action_point: Union[ActionPoint, None] = None
		self.__action_point_origin: Union[ActionPoint, None] = None

		self.__state = ActionStatus.IDLE
	
	def __parsed__(self, context):
		for i in range(len(self.children)):
			child = self.children[i]

			# Non native action children
			if type(child) == Action and not child.native:
				self.children[i] = ActionGroup.parse_file(
					context.get("folder") + child.name + ".xml",
					context = context
				)


	@property
	def state(self) -> int:
		return self.__state

	@state.setter
	def state(self, state: int):
		'''
			Set action block state, in case the action is requested to be paused
			or finished, it first check all possible action inside the block is
			paused or finished, and then pause parent, or do not pause
		'''
		# TODO handle BEST action course
		# If we set action as something else than waiting
		# we do a ascending recursion
		if state != ActionStatus.IDLE:
			# If we encounter a pause, the new state become paused
			translate_to_pause: bool = False

			# Check all children make the state valid
			for next in self.children:
				# Action to be performed found
				if next.state == ActionStatus.IDLE:
					# deny state
					return
				elif next.state == ActionStatus.PAUSED:
					# If one action is done, it cannot be finished or impossible
					translate_to_pause = True

				# If it's a unfinished sync action
				if self.type == ActionGroup.ORDERED and next.state != ActionStatus.DONE:
					# No need to proceed further, we can apply state
					break

			# Apply state
			# TODO check
			super().state.fset(self, ActionStatus.PAUSE if translate_to_pause else state)
		
		# Otherwise if we resume action
		elif self.state == ActionStatus.PAUSED:
			self.__state = ActionStatus.IDLE

			# We perform a descending unpause without questions
			for next in self.children:
				if next.state == ActionStatus.PAUSED:
					next.state = ActionStatus.IDLE
	
	def total_points(self):
		points = self.points

		for child in self.children:
			points += child.total_points()
		
		return points
	
	def add(self, action: Action):
		# Remove action from it's parent
		if action.parent != None:
			action.parent.remove(action)

		# Set ourself as parent
		action.parent = self
		self.children.append(action)
	

	def travel_distance(self, origin: Pose2D) -> float:
		distance = 0
		current_point = origin

		# TODO: compute for unordered group by sorting by max scoring
		for child in self.children:
			child_dist = child.travel_distance(current_point)

			if self.type == ActionGroup.BEST:
				# Best -> minimal distance
				if child_dist < distance:
					distance = child_dist
			else:
				distance += child_dist
				current_point = child.action_point().end
		
		return distance
	

	def action_point(self, origin: Pose2D) -> ActionPoint:
		# Check if previously saved for this origin
		if self.__action_point != None and self.__action_point_origin == origin:
			return self.__action_point

		start: Pose2D = None
		current: Pose2D = origin

		for child in self.children:
			child_action_point = child.action_point(current)
			current = child_action_point.end

			if start == None:
				start = child_action_point.start
		
		# Save action point
		self.__action_point = ActionPoint()
		self.__action_point.start = start
		self.__action_point.end = current

		# And it's origin
		self.__action_point_origin = origin

		return self.__action_point
	
	def __str__(self):
		return "[{}] points={}".format(self.name, self.total_points())
