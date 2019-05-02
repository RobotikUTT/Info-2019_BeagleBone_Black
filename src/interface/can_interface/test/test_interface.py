#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from interface_msgs.msg import CanData
from can_interface import FrameList, DeviceList

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
		pass
	

	def test_frames_input(self):
		for frame_name in self.frames.by_name:
			frame = self.frames.by_name[frame_name]

			data = CanData()
			data.type = frame.name
			
			self.output_pub.publish(data)		

			response = self.bus.recv(timeout=2)
			
			self.assertNotEqual(response, None, "response received")
			self.assertEqual(response.arbitration_id, self.devices.by_name[frame.dest], "right frame received")
			self.assertEqual(response.data[0], frame.id, "right frame received")


if __name__ == '__main__':
	rostest.rosrun('can_interface', 'test_interface', TestInterface, sys.argv)
