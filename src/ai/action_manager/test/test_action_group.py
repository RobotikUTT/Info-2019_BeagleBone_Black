#!/usr/bin/python3
import sys
import unittest
import math

import rospy
import rostest

from action_manager import Action
from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D


class TestOrderedActionGroup(unittest.TestCase):
	def setUp(self):
		self.robot_position = Pose2D()
		self.robot_position.x = 0
		self.robot_position.y = 0

		self.first_position = Pose2D()
		self.first_position.x = 30
		self.first_position.y = 10

		self.second_position  = Pose2D()
		self.second_position.x = 90
		self.second_position.y = 60

		self.third_position  = Pose2D()
		self.third_position.x = 90
		self.third_position.y = 60

		self.action_point = ActionPoint()
		self.action_point.start = self.first_position
		self.action_point.end = self.second_position

	def test_set_state(self):
		self.assertTrue(False, "set state")
	
	def test_action_point(self):
		self.assertTrue(False, "get action point")
	
	def test_travel_distance(self):
		self.assertTrue(False, "get travel distance")
	
	def test_priority(self):
		self.assertTrue(False, "get priority")

	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_ordered_action_group', TestOrderedActionGroup, sys.argv)