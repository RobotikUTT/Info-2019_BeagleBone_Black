#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from time import time, sleep

from action_manager import PerformClient, Action
from action_manager.perform_client import PerformException
from ai_msgs.msg import ActionStatus
from geometry_msgs.msg import Pose2D

class TestPerformClient(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_perform_client', anonymous=True)

		self.action_state = ActionStatus.IDLE

		self.pc = PerformClient("test_perform_client", "action_manager")
		self.pc.on_action_returns = self.action_returns

	def action_returns(self, state, result):
		self.action_state = result.state

	def test_perform_success(self):
		self.action_state = ActionStatus.IDLE
		action = Action.parse_string("""
			<sleep><duration>0.5</duration></sleep>
		""")

		self.pc.perform_action(action, Pose2D())

		start = time()
		while not rospy.is_shutdown() and time() - start < 1 \
			and self.action_state == ActionStatus.IDLE:

			sleep(0.1)
		
		self.assertAlmostEqual(time() - start, 0.5, delta=0.2, msg="pause for the right amount of time")
		self.assertEqual(self.action_state, ActionStatus.DONE, "right action status")

	def test_perform_paused(self):
		self.action_state = ActionStatus.IDLE
		action = Action.parse_string("<fail />")

		self.pc.perform_action(action, Pose2D())

		start = time()
		while not rospy.is_shutdown() and time() - start < 0.2 \
			and self.action_state == ActionStatus.IDLE:
			sleep(0.1)

		self.assertEqual(self.action_state, ActionStatus.PAUSED, "right action status")


	def test_perform_not_found(self):
		action = Action.parse_string("<no_action_bound />")

		# TODO assert fail
		with self.assertRaises(PerformException):
			self.pc.perform_action(action, Pose2D())

if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_perform_client', TestPerformClient, sys.argv)
