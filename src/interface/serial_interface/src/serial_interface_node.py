#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in interface_description
import rospy

from xml_class_parser import Parsable, Bind, Context

from args_lib.argumentable import Argumentable
from node_watcher import NodeStatusHandler

from rospy import Publisher, Subscriber

from ai_msgs.msg import NodeStatus
from interface_msgs.msg import CanData

from can_interface import Frame, FrameList, DeviceList, Param
from can_interface.param import MissingParameterException

from typing import Type, Dict, List

import time
import threading
import traceback

import sys
import serial

class FakeFrameForHurryPurpose:
	def __init__(self, data):
		self.data = data

class SerialBus():

	def __init__(self, device, handler):
		self.enabled = True
		self.handler = handler
		try:
			self.serial = serial.Serial(port=device, baudrate=57600)
		except serial.serialutil.SerialException as e:
			rospy.logerr("Unable to open serial port on {}".format(device))
			self.enabled = False
			return

		self.serial_input_thread = threading.Thread(
			name="serial_input_{}".format("device"),
			target=self.wait_for_serial_message
		)

		# Start thread
		self.serial_input_thread.start()

	def wait_for_serial_message(self):
		'''
			Loop in CAN bus to read data
		'''

		try:
			while not rospy.is_shutdown():
				self.read_serial_message()
		except Exception as e:
			self.serial.close()
			
			if isinstance(e, serial.SerialException):
				rospy.logwarn("Serial reception interrupted")
			else:
				raise e

	def read_byte(self):
		return int.from_bytes(self.serial.read(), byteorder="big")
	
	def read_serial_message(self):
		"""
			Callback from messages from can
		"""

		frame_id = self.read_byte()

		if frame_id not in self.handler.frames.by_id:
			rospy.logerr("received unhandled frame of id {}".format(frame_id))
			return
		
		# Get frame type
		frame_type = self.handler.frames.by_id[frame_id]
		
		rospy.logdebug("received frame from serial of type {}".format(frame_type.name))

		# Handle pong data
		if frame_type.name == "pong":
			address: int = self.read_byte()
			status: int = self.read_byte()

			# Set status if in devices
			if address in self.handler.devices.by_id:
				self.handler.set_node_status(self.handler.devices.by_id[address], "board", NodeStatus.READY)
			else:
				rospy.logerr("received pong frame for an unknown device of id {}".format(address))
		else:
			# Read data
			data = [frame_id]
			size = frame_type.size()

			while len(data) < size:
				data.append(self.read_byte())

			# Create a fake frame object :)
			frame = FakeFrameForHurryPurpose(data)

			# Create message
			message = CanData()
			try:
				message.params = frame_type.extract_frame_data(frame).to_list()
			except IndexError:
				rospy.logerr("received incomplete frame of type {}, ignoring".format(frame_type.name))
				return

			message.type = frame_type.name

			# Publish
			self.handler.publisher.publish(message)
				

class SerialInterfaceNode(NodeStatusHandler):
	def __init__(self):
		super().__init__()
		self.set_node_status("serial_interface", "interface", NodeStatus.INIT)

		# Parse devices
		self.devices: DeviceList = DeviceList.parse_file(
			"can/devices.xml",
			package="interface_description"
		)

		# Then frames
		self.frames: Dict[str, Frame] = FrameList.parse_file(
			"can/frames.xml",
			package="interface_description",
			context=Context(devices=self.devices)
		)

		# Create serial bus bus with given interface
		self.serials = [
			SerialBus("/dev/ttyUSB0", self),
			SerialBus("/dev/ttyUSB1", self)
		]
		
		self.subscriber: Subscriber = Subscriber("/can_interface/out", CanData, self.on_ros_message)
		self.publisher: Publisher = Publisher("/can_interface/in", CanData, queue_size=10)

		# Declare can_interface ready
		self.set_node_status("serial_interface", "interface", NodeStatus.READY)

	def on_ros_message(self, message):
		"""
			Handle message from ROS and build a frame from it
		"""
		
		rospy.logdebug("received frame to send of type {}".format(message.type))

		# Get message params as argumentable
		values = Argumentable().from_list(message.params)
		frame_type = self.frames.by_name[message.type]

		# Prepare data array and set frame type
		data = frame_type.get_frame_data(values)

		if data is not None:
			for bus in self.serials:
				if bus.enabled:
					bus.serial.write(bytes(data[:frame_type.size()]))
					bus.serial.flush()

	def close(self):
		for bus in self.serials:
			if bus.enabled:
				bus.serial.close()


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('serial_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = SerialInterfaceNode()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
		
		node.close()
		sys.exit()
	except rospy.ROSInterruptException:
		pass