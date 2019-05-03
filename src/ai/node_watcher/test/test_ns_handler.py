#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

class TestNSHandler(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_ns_handler', anonymous=True)

	def runTest(self):
		self.fail("not implemented")


if __name__ == '__main__':
	rostest.rosrun('node_watcher', 'test_ns_handler', TestNSHandler, sys.argv)
