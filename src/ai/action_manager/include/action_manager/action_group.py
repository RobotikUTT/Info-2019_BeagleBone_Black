from .action import Action, ActionChoice

import rospy

from ai_msgs.msg import ActionPoint, ActionStatus
from geometry_msgs.msg import Pose2D
from args_lib.msg import Argument

from typing import List, Union

from xml_class_parser import Parsable, Bind, BindDict, Enum, BindList, Context

@Parsable(
	name = "group",
	attributes = {
		"type": Enum(values = ["ordered", "unordered", "best"]),
		"fail": Enum(values = ["restart", "continue", "postpone"]),
		"points": str
	},
	children = [
		BindList(to="children", type=Action),
		BindList(to="children", type=Parsable.SELF),
		BindList(to="arguments", type=Argument) # groups takes arguments for children
	]
)
class ActionGroup(Action):
	ORDERED = "ordered" # actions done in order
	UNORDERED = "unordered" # actions done in any order
	BEST = "best" # only the best action is made

	FAIL_RESTART = "restart" # retry all actions in group
	FAIL_CONTINUE = "continue" # retry children action 
	FAIL_POSTPONE = "postpone" # pause action

	def __init__(self):
		super().__init__()

		self.fail = ActionGroup.FAIL_POSTPONE
		self.type: str = ActionGroup.ORDERED
		self.children: List[Action] = []
	
		self.__action_point: Union[ActionPoint, None] = None
		self.__action_point_origin: Union[ActionPoint, None] = None

		self.__state = ActionStatus.IDLE
		self.__best_child = None

	def __parsed__(self, context: Context):
		super().__parsed__(context)

		for i in range(len(self.children)):
			child = self.children[i]
			
			# Non native action children
			if type(child) == Action and not child.native:
				# Recover list format
				child.arguments = child.arguments.to_list()

				# Create adequate context
				child_context = Context(**context.params)
				child_context.parent = child
				
				# Parse children with context
				self.children[i] = ActionGroup.parse_file(
					context.get("folder") + child.name + ".xml",
					context = child_context
				)
			
			# Set children parent
			self.children[i].parent = self

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

		# handle failure states
		if state != ActionStatus.DONE and state != ActionStatus.IDLE:
			if self.fail == ActionGroup.FAIL_CONTINUE:
				# Deny children pausing
				state = ActionStatus.IDLE
				self.__state = ActionStatus.PAUSED

			elif self.fail == ActionGroup.FAIL_RESTART:
				rospy.logwarn("restart action group handling is not supported yet")
				"""# Restart all children
				stack = [] + self.children
				while len(stack) > 0:
					next = stack.pop()
					print(next)
					if isinstance(next, ActionGroup):
						stack.extend(next.children)

					# force state to restart everything
					next.__state = ActionStatus.IDLE
				
				self.__state = state"""

		# Best action action course
		if state != ActionStatus.IDLE and self.type == ActionGroup.BEST:
			self.__state = state

			# Propagate
			if self.parent != None:
				self.parent.state = self.state
		
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

				# If it's a unfinished ordered action group
				if self.type == ActionGroup.ORDERED and next.state != ActionStatus.DONE:
					# No need to proceed further, we can apply state
					break

			# Apply state
			self.__state = ActionStatus.PAUSED if translate_to_pause else state

			# If not resuming, we propagate to parent
			if self.state != ActionStatus.IDLE and self.parent != None:
				self.parent.state = self.state
		
		# Otherwise if we resume action
		elif self.state == ActionStatus.PAUSED:
			self.__state = ActionStatus.IDLE

			# We perform a descending unpause without questions
			for next in self.children:
				if next.state == ActionStatus.PAUSED:
					next.state = ActionStatus.IDLE
	
	def set_side(self, side):
		# Propagate side to children
		for sub in self.children:
			sub.set_side(side)

	def total_points(self):
		points = self.points

		if self.type == ActionGroup.BEST:
			best = self.best_child()
			return points + (best.total_points() if best is not None else 0)

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
	
	def priority(self, origin: Pose2D):
		if self.type == ActionGroup.BEST:
			best_child = self.best_child()

			if best_child is None:
				return math.inf
			else:
				# TODO travel distance
				return (self.points + best_child.total_points()) ** 2 / best_child.travel_distance(origin)
		else:
			return super().priority(origin)

	def travel_distance(self, origin: Pose2D) -> float:
		if self.type == ActionGroup.BEST:
			best = self.best_child()
			return best.travel_distance(origin) if best is not None else 0

		distance = 0
		current_point = origin

		for child in self.children:
			child_dist = child.travel_distance(current_point)
			
			distance += child_dist
			current_point = child.action_point(current_point).end
		
		return distance
	

	def action_point(self, origin: Pose2D) -> ActionPoint:
		# Check if previously saved for this origin
		if self.__action_point != None and self.__action_point_origin == origin:
			return self.__action_point

		if self.type == ActionGroup.BEST:
			return self.best_child().action_point(origin)

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

	def get_optimal(self, robot_pos: Pose2D) -> ActionChoice:
		"""
			Returns the optimal choice of native action from this action.

			It takes in account the subactions and adapt based on group
			mode
		"""
		choice = ActionChoice()
		choice.score = -1 # ensure actions with 0 priority are taken into account

		if self.state != ActionStatus.IDLE:
			return choice

		# Try all subactions
		for next in self.children:
			next_choice = next.get_optimal(robot_pos)

			# If the action has a better score
			if next_choice.score > choice.score:
				# We choose it !
				choice = next_choice
			

			# If we hit a unfinished action in an ordered group, it must
			# be performed in priority
			if self.type == ActionGroup.ORDERED and next.state != ActionStatus.DONE:
				# Choose this action and quit loop
				choice = next_choice
				break
		

		# As this is a group, we use the group's priority instead of the
		# atomic action's priority
		choice.score = self.priority(robot_pos)
		return choice

	def best_child(self) -> Action:
		"""
			Returns children with max points
		"""
		if self.__best_child is not None:
			return self.__best_child

		best_child = None
		best_prio = 0
		for child in self.children:
			prio = child.total_points()
			if prio > best_prio:
				best_prio = prio
				best_child = child

		# Cache it
		self.__best_child = best_child
		return best_child

	def __str__(self):
		sublines = [(" ╟──" + str(child)) for child in self.children]

		if len(sublines) > 0:
			sublines[-1] = " ╙" + sublines[-1][2:]
			part = sublines[-1].split("\n")

			for i in range(len(part) - 1):
				part[i + 1] = "   " + part[i + 1]

			sublines[-1] = "\n".join(part)

		for j in range(len(sublines) - 1):
			part = sublines[j].split("\n")

			for i in range(len(part) - 1):
				part[i + 1] = " ║  " + part[i + 1]
			
			sublines[j] = "\n".join(part)


		return "{} points={}\n".format(
				self.color("[{}]".format(self.type)), self.total_points()
			) + "\n".join(sublines)
