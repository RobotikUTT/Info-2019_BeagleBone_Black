from rospy import Subscriber, Service
import rospy
from actionlib import SimpleActionServer

from ai_msgs.msg import ActionPoint, Argument, ActionStatus, \
	PerformResult, PerformGoal, PerformAction

from ai_msgs.srv import ComputeActionPointRequest, ComputeActionPointResponse
from interface_msgs.msg import StmDone, Point

from node_watcher import Node
from typing import List, Dict

from abc import ABC, abstractmethod

def get_action_point_service(name: str) -> str:
	return "/action/{}/actionpoint".format(name)

def get_action_server(name: str) -> str:
	return "/action/{}".format(name)

'''
  Represent an performer for a specific action, it advertise a service for
  predicting position and an action server to run the action.
  
  It provide simple interface for creating robot action, and adef tedious ROS objects
  manipulation.
'''
class ActionPerformer(Node, ABC):
	def __init__(self, name: str):
		super().__init__()
		self.name = name

		# Service for action point computation
		self._action_point_server = Service(
			get_action_point_service(name),
			self._compute_action_point
		)

		# Action server for this action
		self._action_server = SimpleActionServer(
			get_action_server(name),
			PerformAction
		)

		self._action_server.register_goal_callback(self._on_goal)
		self._action_server.register_preempt_callback(self._on_preempt)
		self._action_server.start()


	# Function managing the action
	def action_performed(self):
		'''Function to call when the action is performed'''
		# Create result message
		result = PerformResult()
		result.status.state_code = ActionStatus.DONE

		# Send back to client
		self._action_server.set_succeeded(result)

	def action_paused(self):
		'''Fonction to call when the action is paused'''
		result = PerformResult()
		result.status.state_code = ActionStatus.PAUSED

		# Send back to client
		self._action_server.set_succeeded(result)
		
	def _args_to_dict(self, args: List[Argument]) -> Dict[str, float]:
		result: Dict[str, float] = {}

		for arg in args:
			result[arg.name] = arg.value

	# Hidden handlers
	def _compute_action_point(self, request: ComputeActionPointRequest) -> ComputeActionPointResponse:
		# Extract data from request and call performer's function
		result: ActionPoint = self.compute_action_point(
			self._args_to_dict(request.args),
			request.robot_pos
		)

		response = ComputeActionPointResponse()
		response.action_point = result

		return response

	def _on_goal(self):
		if self._action_server.is_new_goal_available():
			goal: PerformGoal = self._action_server.accept_new_goal()

			# run action with given args
			self.start(self._args_to_dict(goal.args))

	def _on_preempt(self):
		self.cancel()
		self._action_server.set_preempted()


	# Function to be defined by inherited actions
	@abstractmethod
	def compute_action_point(self, args: Dict[str, float], robot_pos: Point) -> ActionPoint:
		pass
	
	@abstractmethod
	def start(self, args: Dict[str, float]):
		pass

	def cancel(self):
		pass
