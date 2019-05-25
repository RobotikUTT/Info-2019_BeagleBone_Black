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

class TestBestActionGroup(TestOrderedActionGroup):
	def setUp(self):
		super().setUp("best")

	def test_set_state_idle(self, state=ActionStatus.PAUSED, assert_idle=True):
		root = self.example()

		for child in [root.children[0], root.children[1], root.children[2].children[0]]:
			child.state = state
			self.assertEqual(root.state, state)

			root.state = ActionStatus.IDLE
			if assert_idle:
				self.assertEqual(child.state, ActionStatus.IDLE)
			
	def test_set_state_paused(self):
		# defined in same test
		self.test_set_state_idle()

	def test_set_state_error(self):
		self.test_set_state_idle(ActionStatus.ERROR, False)

	def test_set_state_done(self):
		root = self.example()
		root.children[1].state = ActionStatus.DONE

		# Assert is done
		self.assertEqual(root.state, ActionStatus.DONE)


	def test_action_point(self):
		pass

	def test_priority(self):
		root = self.example()

		# define action point sequences
		root.children[0].action_point = action_point_fun(2, 3)
		root.children[1].action_point = action_point_fun(4, 5)
		root.children[2].children[0].action_point = action_point_fun(6, 7)

		# assert points used here
		self.assertEqual(root.children[0].total_points(), 0)
		self.assertEqual(root.children[1].total_points(), 1)
		self.assertEqual(root.children[2].total_points(), 5)

		# assert right child priority
		self.assertEqual(root.children[0].priority(self.robot_position), 0)
		self.assertEqual(root.children[1].priority(self.robot_position), 1 / math.sqrt(9 + 16))
		self.assertEqual(root.children[2].priority(self.robot_position), 5 * 5 / math.sqrt(25 + 36))

		# assert right parent priority
		self.assertEqual(root.priority(self.robot_position), max(1 / math.sqrt(9 + 16), 5 * 5 / math.sqrt(25 + 36)))
	
	def test_travel_distance(self):
		pass
	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_best_action_group', TestBestActionGroup, sys.argv)
