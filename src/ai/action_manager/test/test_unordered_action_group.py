#!/usr/bin/python3
import sys
import unittest
import math

import rospy
import rostest

from action_manager import Action, ActionGroup
from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D

from test_ordered_action_group import action_point_fun, TestOrderedActionGroup

class TestUnorderedActionGroup(TestOrderedActionGroup):
	def setUp(self):
		super().setUp("unordered")

	def test_set_state_idle(self):
		root = self.example()
		root.children[0].state = ActionStatus.DONE
		root.children[1].state = ActionStatus.IDLE
		sub = root.children[2]
		sub.children[0].state = ActionStatus.PAUSED

		# Assert is paused
		self.assertEqual(root.state, ActionStatus.IDLE)
		self.assertEqual(sub.state, ActionStatus.PAUSED)

		# Try to restore
		sub.state = ActionStatus.IDLE

		# Assert idle restored
		self.assertEqual(sub.state, ActionStatus.IDLE)
		self.assertEqual(sub.children[0].state, ActionStatus.IDLE)
	
	def test_set_state_paused(self, state=ActionStatus.PAUSED):
		root = self.example()

		# Test pause only when all subactions paused
		for i in range(2):
			root.children[i].state = state
			self.assertEqual(root.state, ActionStatus.IDLE)

		root.children[2].children[0].state = state

		self.assertEqual(root.state, state)

	def test_set_state_error(self):
		self.test_set_state_paused(ActionStatus.ERROR)
	
	def test_travel_distance(self):
		raise NotImplementedError()


	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_unordered_action_group', TestUnorderedActionGroup, sys.argv)
