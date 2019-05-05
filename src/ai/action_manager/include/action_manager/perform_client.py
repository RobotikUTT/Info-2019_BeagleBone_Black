
from node_watcher import Node
from .action import Action
from.action_group import ActionGroup
from .util import get_action_server, get_action_node_path

import rospy
import actionlib
from ai_msgs.msg import PerformAction, PerformGoal, ActionStatus
from geometry_msgs.msg import Pose2D

from typing import Union

class PerformClient(Node):
	def __init__(self, name: str, package: str):
		super().__init__(name, package)

		self.client: Union[actionlib.SimpleActionClient, None] = None

	def on_finished(self):
		pass
	
	def on_paused(self):
		pass
	
	def perform_action(self, action: Action, position: Pose2D):
		client = actionlib.SimpleActionClient(get_action_server(action.name), PerformAction)
		client.wait_for_server()

		goal = PerformGoal()
		goal.arguments = action.arguments.to_list()
		goal.robot_pos = position

		client.send_goal(goal, done_cb=self.done_callback)

	def done_callback(self, state, result):
		if result.status == ActionStatus.DONE:
			self.on_finished()
		else:
			self.on_paused()
	
	def cancel_action(self):
		if self.client != None:
			self.client.cancel_goal()
			self.on_paused()
	
	def save_required(self, action: Action):
		'''
			Register given action's performer or it's dependencies (if group) as required
		'''
		# Try to cast as block to add all subactions requirements
		if isinstance(action, ActionGroup):
			for child in action.children:
				self.save_required(child)

		else:
			performer = get_action_node_path(action.name)

			# Check if the performer is not already required
			if self.is_required(performer):
				return

			# Add to the list otherwise
			self.require(action.name, "action", False)

