#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from interface_msgs.msg import CanData
from can_interface import FrameList, DeviceList
from action_manager import Argumentable

import can

class TestInterface(unittest.TestCase):

	def setUp(self):
		rospy.init_node('test_interface', anonymous=True)

		# Create a bus can
		can.rc['interface'] = 'socketcan_ctypes'
		self.bus = can.Bus("vcan0")

		self.input_sub = rospy.Subscriber("/can_interface/in", CanData, self.message_received)
		self.output_pub = rospy.Publisher("/can_interface/out", CanData)

		self.devices = DeviceList.parse_file("can/devices.xml", "interface_description")
		self.frames = FrameList.parse_file("can/frames.xml", "interface_description", context={"devices": self.devices})

		# Wait for a connection
		poll_rate = rospy.Rate(100)
		while self.output_pub.get_num_connections() == 0 and not rospy.is_shutdown():
			poll_rate.sleep()

	def message_received(self, message):
		print(message)

	def test_ping(self):
		self.assertEqual(True, False)
	

	def test_frames_input(self):
		val = 21

		for frame_name in self.frames.by_name:
			if frame_name == "pong":
				continue
				
			frame = self.frames.by_name[frame_name]

			# Prepare data to send
			req = CanData()
			req.type = frame.name

			# Put all values in message
			initial = Argumentable()
			for param in frame.params:
				initial.set(param.name, val)
				val = (val + 12) % 256
			
			req.params = initial.to_list()

			self.output_pub.publish(req)

			response = self.bus.recv(timeout=1)
			print("GOGO", req, "OAOAOOAOAO", response)
			self.assertNotEqual(response, None, "response received")
			self.assertEqual(response.arbitration_id, self.devices.by_name[frame.dest], "right frame received")
			self.assertEqual(response.data[0], frame.id, "right frame received")

			# Check received data
			received = frame.extract_frame_data(response)

			# Assert equal to initial
			for param in frame.params:
				self.assertEqual(initial.get(param), received.get(param), "received data intact")

	def test_invalid_frame_ignore(self):
		self.assertEqual(True, False)

if __name__ == '__main__':
	rostest.rosrun('can_interface', 'test_interface', TestInterface, sys.argv)
