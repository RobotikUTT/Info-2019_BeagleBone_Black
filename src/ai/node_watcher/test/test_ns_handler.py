#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest
import time

from rospkg import RosPack
from node_watcher import NodeStatusHandler
from ai_msgs.msg import NodeStatus

class TestNSHandler(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_ns_handler', anonymous=True)

		self.ns_handler = NodeStatusHandler()

		self.wait = -1
	
	def wait_over(self, success):
		self.wait = success

	def test_ns_change(self):
		"""
			Test status alteration
		"""
		self.assertEqual(self.ns_handler.get_node_status("test", "test_pkg").state_code, NodeStatus.UNKNOW)

		for status in vars(NodeStatus):
			if not isinstance(status, int):
				continue

			if status == NodeStatus.ERROR:
				self.ns_handler.set_node_status("test", "test_pkg", status, 42)
				self.assertEqual(self.ns_handler.get_node_status("test", "test_pkg").state_code, status)
				self.assertEqual(self.ns_handler.get_node_status("test", "test_pkg").error_code, 42)
			else:
				self.ns_handler.set_node_status("test", "test_pkg", status)
				self.assertEqual(self.ns_handler.get_node_status("test", "test_pkg").state_code, status)
			
	def test_node_await(self):
		self.wait = -1
		
		self.ns_handler.reset_requirements()
		self.ns_handler.require("test_await", "test_pkg")
		self.ns_handler.set_wait_callback(self.wait_over)
		self.ns_handler.wait_for_nodes(1)

		self.ns_handler.set_node_status("test_await", "test_pkg", NodeStatus.READY)

		start = time.time()
		while not rospy.is_shutdown() and time.time() - start < 1 and self.wait == -1:
			rospy.sleep(0.1)
		
		self.assertTrue(self.wait, "wait over")
	
	def test_node_await_fail(self):
		self.wait = -1

		start = time.time()

		self.ns_handler.reset_requirements()
		self.ns_handler.require("test_await_fail", "test_pkg")
		self.ns_handler.set_wait_callback(self.wait_over)
		self.ns_handler.wait_for_nodes(0.2)

		while not rospy.is_shutdown() and time.time() - start < 1 and self.wait == -1:
			rospy.sleep(0.1)
		
		self.assertEqual(self.wait, False, "wait over with result fail")

		

if __name__ == '__main__':
	rostest.rosrun('node_watcher', 'test_ns_handler', TestNSHandler, sys.argv)
