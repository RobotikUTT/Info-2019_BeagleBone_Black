#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest
import tempfile
import rospkg

from action_manager import Action, ActionGroup

class TestParsing(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_action_parsing', anonymous=True)

	def test_action_parsing(self):
		parsed = Action.parse_string("""
			<parse native="true" points="34">
				<file>yes</file>
				<smthg>34</smthg>
			</parse>
		""")

		self.assertEqual(parsed.name, "parse", "name parsed")
		self.assertEqual(parsed.native, True, "native parsed")
		self.assertEqual(parsed.repeat, None, "native parsed")
		self.assertEqual(parsed.points, 34)

		self.assertEqual(parsed.arguments.get("file"), "yes")
		self.assertEqual(parsed.arguments.get("smthg", int), 34)

	def test_action_group_parsing(self):
		parsed = ActionGroup.parse_string("""
			<group>
				<group>
					<o_o native='true' />
				</group>
				<run native='true' />
				<yay native='true' />
			</group>
		""", context={"folder": "./"})

		# Test groups size
		self.assertEqual(len(parsed.children), 3)

		self.assertIsInstance(parsed.children[0], ActionGroup, "saved in order")
		self.assertEqual(len(parsed.children[0].children), 1)

		self.assertIsInstance(parsed.children[1], Action)
		self.assertEqual(parsed.children[1].name, "run", "saved in order")

		self.assertIsInstance(parsed.children[2], Action)
		self.assertEqual(parsed.children[2].name, "yay", "saved in order")

	def test_action_inclusion(self):
		source_folder = rospkg.RosPack().get_path("action_manager") + "/test/actions/"
		parsed = ActionGroup.parse_string("""
			<group><move_a_lot native='false' /></group>
		""", context={"folder": source_folder})

		self.assertIsInstance(parsed.children[0], ActionGroup)
		self.assertEqual(len(parsed.children[0].children), 2)
		self.assertEqual(len(parsed.children[0].children[1].children), 2)
	
	def test_argument_passing(self):
		"""
			test
			<group native="false">
				<arg1>value</arg1> -> passed to context for children actions
			</group>
		"""
		self.fail("not implemented")

	def test_include_argument_passing(self):
		"""
			test
			<action native="false">
				<arg1>value</arg1> -> passed to context
			</action>
		"""
		self.fail("not implemented")
		
	def test_alias_usage(self):
		"""
			test
			<action native="false">
				<arg1>{destination}</arg1> -> map destination to arg1
			</action>
		"""
		self.fail("not implemented")

if __name__ == '__main__':
	rostest.rosrun('action_manager', 'test_action_parsing', TestParsing, sys.argv)
