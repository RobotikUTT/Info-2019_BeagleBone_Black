
from node_watcher import Node
from .action import Action
from.action_group import ActionGroup
from .util import get_action_server, get_action_node_path

import rospy
import actionlib
from ai_msgs.msg import PerformAction, PerformGoal, ActionStatus
from geometry_msgs.msg import Pose2D

from typing import Union

class PerformException(Exception):
	pass

class PerformClient(Node):
	def __init__(self, name: str, package: str):
		super().__init__(name, package)

		self.client: Union[actionlib.SimpleActionClient, None] = None

	
	def perform_action(self, action: Action, position: Pose2D):
		# Create client
		client = actionlib.SimpleActionClient(get_action_server(action.name), PerformAction)

		# Connect
		if client.wait_for_server(rospy.Duration(0.8)):
			goal = PerformGoal()
			goal.arguments = action.arguments.to_list()
			goal.robot_pos = position

			client.send_goal(goal, done_cb=self.on_action_returns)
		else:
			raise PerformException("unable to reach server")

	def on_action_returns(self, state, result):
		pass
		
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

