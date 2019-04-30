#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from can_interface import DevicesHandler

class TestParsing(unittest.TestCase):
	def setUp(self):
		pass

	def test_parsing(self):
		content = """
			<devices>
				<device id='3' name=''>
					<input />
					<output />
				</device>
			</devices>
		"""

		handler = DevicesHandler.parse_string(content)





if __name__ == '__main__':
	rostest.rosrun('can_interface', 'test_parsing', TestParsing, sys.argv)
