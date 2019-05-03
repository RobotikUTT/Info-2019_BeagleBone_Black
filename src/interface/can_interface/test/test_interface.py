#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest
import time

from interface_msgs.msg import CanData
from ai_msgs.msg import NodeStatus

from can_interface import FrameList, DeviceList
from action_manager import Argumentable
from node_watcher import NodeStatusHandler

import can

class TestInterface(unittest.TestCase):

	def setUp(self):
		rospy.init_node('test_interface', anonymous=True)

		# Message received
		self.messages = []

		# Create a bus can
		can.rc['interface'] = 'socketcan_ctypes'
		self.bus = can.Bus("vcan0")

		self.input_sub = rospy.Subscriber("/can_interface/in", CanData, self.message_received)
		self.output_pub = rospy.Publisher("/can_interface/out", CanData, queue_size=10)

		self.devices = DeviceList.parse_file("can/devices.xml", "interface_description")
		self.frames = FrameList.parse_file("can/frames.xml", "interface_description", context={"devices": self.devices})

		# Wait for a connection
		poll_rate = rospy.Rate(100)
		while self.output_pub.get_num_connections() == 0 and not rospy.is_shutdown():
			poll_rate.sleep()

	def tearDown(self):
		self.bus.shutdown()

	def message_received(self, message):
		self.messages.append(message)

	def test_pong(self):
		ns_handler = NodeStatusHandler()

		for device_id in self.devices.by_id:
			frame = self.frames.by_name["pong"]

			# Check that board is not up
			self.assertEqual(ns_handler.get_node_status(
				self.devices.by_id[device_id],
				"board"
			).state_code, NodeStatus.UNKNOW, "initially down")

			# Send pong message
			self.bus.send(can.Message(
				data=[frame.id, device_id, 0],
				dlc=3,
				arbitration_id=self.devices.by_name[frame.dest],
				is_extended_id=False
			))
			
			start = time.time()
			poll_rate = rospy.Rate(100)
			while time.time() - start < 1.1 and not rospy.is_shutdown():
				# Wait for it to be ready
				if ns_handler.get_node_status(
					self.devices.by_id[device_id],
					"board"
				).state_code == NodeStatus.READY:
					break

				poll_rate.sleep()

			self.assertLess(
				time.time() - start, 1, "/board/{} did not get ready after pong"
					.format(self.devices.by_id[device_id]))

	def test_frame_output(self):
		val = 27

		for frame_name in self.frames.by_name:
			if frame_name == "pong":
				continue
				
			frame = self.frames.by_name[frame_name]

			# Prepare data to send
			can_req = can.Message(
				data=bytearray([frame.id] + [(val + 7 * i) % 256 for i in range(7)]),
				arbitration_id=self.devices.by_name[frame.dest],
				dlc=frame.size(),
				is_extended_id=False
			)
			val += 78

			# Send data
			self.bus.send(can_req)

			# Wait for answer
			start = time.time()
			poll_rate = rospy.Rate(100)
			while time.time() - start < 1.1 and not rospy.is_shutdown()\
				and len(self.messages) == 0:

				poll_rate.sleep()
			
			self.assertLess(time.time() - start, 1, "transmitted in less than a second")

			message = self.messages.pop()

			initial = frame.extract_frame_data(can_req)
			received = Argumentable().from_list(message.params)
			
			self.assertEqual(message.type, frame.name, "right name")
			self.assertEqual(initial.to_list(), received.to_list(), "received same data")

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
				if param.size == 2:
					initial.set(param.name, (val * 267 + 25) % (256 * 256))
				else:
					initial.set(param.name, val)
	
				val = (val + 12) % 256
			
			req.params = initial.to_list()

			self.output_pub.publish(req)

			response = self.bus.recv(timeout=1)

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
