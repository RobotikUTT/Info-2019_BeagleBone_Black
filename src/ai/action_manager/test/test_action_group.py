#!/usr/bin/python3
import sys
import unittest
import math

import rospy
import rostest

from action_manager import Action, ActionGroup
from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D


class TestOrderedActionGroup(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_action_group', anonymous=True)

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

	def test_set_state_idle(self):
		ActionGroup.parse_string("""
			<group>

			</group>
		""")
		raise NotImplementedError()
	
	def test_action_point(self):
		raise NotImplementedError()
	
	def test_travel_distance(self):
		raise NotImplementedError()
	
	def test_priority(self):
		raise NotImplementedError()

	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_ordered_action_group', TestOrderedActionGroup, sys.argv)
