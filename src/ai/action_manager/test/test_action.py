#!/usr/bin/python3
import sys
import unittest
import math

import rospy
import rostest

from action_manager import Action
from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D

class TestAction(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_action', anonymous=True)

		self.robot_position = Pose2D()
		self.robot_position.x = 0
		self.robot_position.y = 0

		self.first_position = Pose2D()
		self.first_position.x = 30
		self.first_position.y = 10

		self.second_position  = Pose2D()
		self.second_position.x = 90
		self.second_position.y = 60

		self.action_point = ActionPoint()
		self.action_point.start = self.first_position
		self.action_point.end = self.second_position

	def test_set_state(self):
		action = Action()

		for state in [ActionStatus.IDLE, ActionStatus.PAUSED, ActionStatus.DONE, ActionStatus.RUNNING]:
			action.state = state
			self.assertEqual(state, action.state, "state is set to {}".format(state))
	
	def test_action_point(self):
		action = Action()

		# Test computation
		self.assertTrue(False, "compute with remote service")
		# TODO

		# Test usage of cache
		action.__action_point = self.action_point
		action.__action_point_origin = self.robot_position

		self.assertEqual(self.action_point, action.action_point(), "does not recompute with cache")
	
	def test_travel_distance(self):
		action = Action()
		action.action_point = lambda org: self.action_point

		self.assertEqual(
			action.travel_distance(self.robot_position),
			# from robot to first to second
			math.sqrt(30 * 30 + 10 * 10) + math.sqrt(60 * 60 + 50 * 50)
		)
	
	def test_priority(self):
		action = Action()
		action.points = 30

		action.travel_distance = lambda org: 78
		self.assertEqual(action.priority(self.robot_position), 900 / 78, "compute regular priority")
		
		# Distance to 0
		action.travel_distance = lambda org: 0
		self.assertEqual(action.priority(self.robot_position), math.inf, "compute priority with travel distance to 0")

	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_action', TestAction, sys.argv)
