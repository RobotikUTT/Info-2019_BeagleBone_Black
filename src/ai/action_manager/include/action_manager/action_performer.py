from rospy import Subscriber, Service
import rospy
from actionlib import SimpleActionServer

# Messages
from ai_msgs.msg import ActionPoint, ActionStatus, \
	PerformResult, PerformGoal, PerformAction
from ai_msgs.srv import ComputeActionPoint, ComputeActionPointRequest, ComputeActionPointResponse
from geometry_msgs.msg import Pose2D
from args_lib.msg import Argument

from node_watcher import Node
from .util import get_action_point_service, get_action_server
from args_lib.argumentable import Argumentable

from typing import List, Dict
from abc import ABC, abstractmethod

'''
  Represent an performer for a specific action, it advertise a service for
  predicting position and an action server to run the action.
  
  It provide simple interface for creating robot action, and adef tedious ROS objects
  manipulation.
'''
class ActionPerformer(Node, ABC):
	def __init__(self, name: str):
		super().__init__(name, "action")
		self.name = name

		# Service for action point computation
		self._action_point_server = Service(
			get_action_point_service(name),
			ComputeActionPoint,
			self._compute_action_point
		)

		# Action server for this action
		self._action_server = SimpleActionServer(
			get_action_server(name),
			PerformAction,
			auto_start=False
		)

		self._action_server.register_goal_callback(self._on_goal)
		self._action_server.register_preempt_callback(self._on_preempt)
		self._action_server.start()


	# Function managing the action
	def returns(self, state = ActionStatus.DONE):
		'''
			Function to call when the action is interrupted
			with a given state
		'''
		# Create result message
		result = PerformResult()
		result.state = state

		# Send back to client
		self._action_server.set_succeeded(result)

	# Hidden handlers
	def _compute_action_point(self, request: ComputeActionPointRequest) -> ComputeActionPointResponse:
		# Extract data from request and call performer's function
		result: ActionPoint = self.compute_action_point(
			Argumentable().from_list(request.args),
			request.robot_pos
		)

		response = ComputeActionPointResponse()
		response.action_point = result

		return response

	def _on_goal(self):
		if self._action_server.is_new_goal_available():
			goal: PerformGoal = self._action_server.accept_new_goal()

			# run action with given args
			self.start(Argumentable().from_list(goal.arguments))

	def _on_preempt(self):
		self.cancel()
		self._action_server.set_preempted()


	# Function to be defined by inherited actions
	@abstractmethod
	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		pass
	
	@abstractmethod
	def start(self, args: Argumentable):
		pass

	def cancel(self):
		pass
