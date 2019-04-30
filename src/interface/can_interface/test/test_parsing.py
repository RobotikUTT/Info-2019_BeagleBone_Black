#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from can_interface import DevicesHandler
from xml_class_parser import Context

class FakeInterface:
	def subscribe(self, a, b):
		pass

class TestParsing(unittest.TestCase):
	def setUp(self):
		pass

	def test_parsing(self):
		content = """
			<devices>
				<device id='3' name=''>
					<input frame="ORDER_SEND_POINT" message="StmDone" topic="STM_SPEED" />
					<output frame="ORDER_SEND_POINT" message="StmDone" topic="STM_SET_SPEED" />
				</device>
			</devices>
		"""
		rospy.init_node('test_parsing', anonymous=True)
		handler = DevicesHandler.parse_string(content, context=Context(interface=FakeInterface()))

if __name__ == '__main__':
	rostest.rosrun('can_interface', 'test_parsing', TestParsing, sys.argv)
