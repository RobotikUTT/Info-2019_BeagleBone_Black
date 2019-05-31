#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from time import time, sleep

from action_manager import PerformClient, Action

from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D

class TestDock_controlPerformer(unittest.TestCase):
	def setUp(self):
		# Init some position
		self.robot_position = Pose2D()
		self.robot_position.x = 0
		self.robot_position.y = 0

		# Some example action point
		self.action_point = ActionPoint()
		self.action_point.start = self.robot_position
		self.action_point.end = self.robot_position

		# Monitored state of return
		self.action_state = ActionStatus.IDLE

		# Perform client
		self.pc = PerformClient("test_dock_control_performer_client", "actions")
		self.pc.on_action_returns = self.action_returns

	def action_returns(self, state, result):
		"""Handle action returns"""
		self.action_state = result.state


	def test_action_point(self):
		"""
			Test action point computation for dock_control performer.
		"""
		# Create move action
		action = Action.parse_string("<dock_control />")
		
		#action.arguments.set("argname", value)

		# Test computation
		self.assertEqual(
			self.action_point, action.action_point(self.robot_position),
			"compute action point with remote service")



	def test_perform_success(self):
		self.action_state = ActionStatus.IDLE
		self.pc.perform_action(Action.parse_string("<dock_control />"), Pose2D())

		# Wait for result
		start = time()
		while not rospy.is_shutdown() and time() - start < 1 \
			and self.action_state == ActionStatus.IDLE:
			sleep(0.1)
		
		# Assert action performed
		self.assertEqual(self.action_state, ActionStatus.DONE, "right action status")


if __name__ == '__main__':
	rospy.init_node('test_dock_control_performer', anonymous=True)
	rostest.rosrun('dock_control', 'actions', TestDock_controlPerformer, sys.argv)
