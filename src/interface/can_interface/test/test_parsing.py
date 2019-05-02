#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from can_interface.devices import Device, DevicesList
from can_interface.frames import FrameList, Frame
from can_interface.param import Param

from xml_class_parser import Context

class TestParsing(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_parsing', anonymous=True)

		self.device_list = DevicesList.parse_string("""
			<devices><device name='me' id='1' /><device name='you' id='2' /><broadcast id='3' name='mlk' /></devices>
		""")

	def test_devices_parsing(self):
		device_list = DevicesList.parse_string("""
			<devices>
				<broadcast id='2' name='everyone' />
				<device id='3' name='yes' />
			</devices>
		""")
		
		self.assertEqual(device_list.by_id, {
			2: "everyone",
			3: "yes"
		}, "dict by id parsed")

		self.assertEqual(device_list.by_name, {
			"everyone": 2,
			"yes": 3
		}, "dict by name parsed")

		self.assertEqual(device_list.broadcast.name, "everyone", "broadcast parsed")
		self.assertEqual(device_list.broadcast.value, 2, "broadcast parsed")

	def test_param_parsing(self):
		param = Param.parse_string("<word name='lol'><mdr v='1' /><lol v='2' /></word>")

		self.assertEqual(param.name, "lol", "name parsing")
		self.assertEqual(param.size, 2)
		self.assertEqual(param.values, {
			"mdr": 1,
			"lol": 2
		})

	def test_frame_parsing(self):
		frame = Frame.parse_string("""
			<frame id="7" name="paola" source="me">
				<word name="wow" />
				<byte name="incredible" />
				<word name="hihi" />
			</frame>
		""", context=Context(devices=self.device_list))

		# Parse properties
		self.assertEqual(frame.source, "me")
		self.assertEqual(frame.dest, "mlk") # broadcast by default

		self.assertEqual(frame.name, "paola")
		self.assertEqual(frame.id, 7)
		self.assertEqual(len(frame.params), 3)

		# Parse childrens
		self.assertEqual(frame.params[0].name, "wow")
		self.assertEqual(frame.params[0].byte_start, 1)
		
		self.assertEqual(frame.params[1].name, "incredible")
		self.assertEqual(frame.params[1].byte_start, 3)

		self.assertEqual(frame.params[2].name, "hihi")
		self.assertEqual(frame.params[2].byte_start, 4)

	def test_frame_list_parsing(self):
		frame_list = FrameList.parse_string("""
			<frames>
				<frame name="go_girl" id="5" />
				<frame name="go_girl_lol" id="1" />
			</frames>
		""", context=Context(devices=self.device_list))

		self.assertIn(5, frame_list.by_id)
		self.assertIn(1, frame_list.by_id)

		self.assertIn("go_girl", frame_list.by_name)
		self.assertIn("go_girl_lol", frame_list.by_name)

		self.assertEqual(frame_list.by_id[5], frame_list.by_name["go_girl"])
		self.assertEqual(frame_list.by_id[1], frame_list.by_name["go_girl_lol"])



if __name__ == '__main__':
	rostest.rosrun('can_interface', 'test_parsing', TestParsing, sys.argv)
