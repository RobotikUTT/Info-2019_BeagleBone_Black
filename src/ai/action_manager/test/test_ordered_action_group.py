#!/usr/bin/python3
import sys
import unittest
import math

import rospy
import rostest

from action_manager import Action, ActionGroup
from ai_msgs.msg import ActionStatus, ActionPoint
from geometry_msgs.msg import Pose2D

def action_point_fun(xf, yf):
	"""Replacement action point generator"""
	def fun(robot_position):
		ret = Pose2D()
		ret.x = robot_position.x * xf
		ret.y = robot_position.y * yf
		p = ActionPoint()
		p.start = robot_position
		p.end = ret
		return p
	return fun

class TestOrderedActionGroup(unittest.TestCase):
	def setUp(self, mode="ordered"):
		rospy.init_node('test_{}_action_group'.format(mode), anonymous=True)

		self.robot_position = Pose2D()
		self.robot_position.x = 1
		self.robot_position.y = 1

		self.__example = """
			<group type="{0}">
				<move native='true' point="0" />
				<move native='true' points="1" />
				<group type="{0}" points="2">
					<move native='true' points="3" />
				</group>
			</group>
		""".format(mode)

	def example(self):
		"""Returns parsed example data"""
		return ActionGroup.parse_string(self.__example)

	def test_set_state_idle(self):
		root = self.example()
		root.children[0].state = ActionStatus.DONE
		root.children[1].state = ActionStatus.DONE
		sub = root.children[2]
		sub.children[0].state = ActionStatus.PAUSED

		# Assert is paused
		self.assertEqual(root.state, ActionStatus.PAUSED)
		self.assertEqual(sub.state, ActionStatus.PAUSED)

		# Try to restore
		root.state = ActionStatus.IDLE

		# Assert idle restored
		self.assertEqual(root.state, ActionStatus.IDLE)
		self.assertEqual(sub.state, ActionStatus.IDLE)
		self.assertEqual(sub.children[0].state, ActionStatus.IDLE)
	
	def test_set_state_paused(self):
		root = self.example()
		root.children[0].state = ActionStatus.PAUSED

		self.assertEqual(root.state, ActionStatus.PAUSED)

	def test_set_state_error(self):
		root = self.example()
		root.children[0].state = ActionStatus.ERROR

		self.assertEqual(root.state, ActionStatus.ERROR)

	def test_set_state_done(self):
		root = self.example()
		root.children[0].state = ActionStatus.DONE
		root.children[1].state = ActionStatus.DONE
		sub = root.children[2]
		sub.children[0].state = ActionStatus.DONE

		# Assert is paused
		self.assertEqual(root.state, ActionStatus.DONE)
		self.assertEqual(sub.state, ActionStatus.DONE)


	def test_action_point(self):
		root = self.example()
		root.children[0].action_point = action_point_fun(2, 3)
		root.children[1].action_point = action_point_fun(4, 5)
		root.children[2].children[0].action_point = action_point_fun(6, 7)

		res = root.action_point(self.robot_position)
		self.assertEqual(res.start, self.robot_position)

		# 3 times same action point computation
		self.assertEqual(res.end.x, 2 * 4 * 6)
		self.assertEqual(res.end.y, 3 * 5 * 7)

		# change a action_point computation (TODO handle changing computation system)
		root.children[1].action_point = action_point_fun(9, 10)

		# assert caching
		res2 = root.action_point(self.robot_position)
		self.assertEqual(res, res2, "cache conservation")
	
	def test_travel_distance(self):
		raise NotImplementedError()


	
		
if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_ordered_action_group', TestOrderedActionGroup, sys.argv)
