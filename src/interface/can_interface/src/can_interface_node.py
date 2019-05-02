#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in interface_description
import rospy

from xml_class_parser import Parsable, Bind, Context
from xml_class_parser.helper import ParsableDict

from action_manager import Argumentable
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

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

class CanInterfaceNode(NodeStatusHandler):
	def __init__(self):
		super().__init__()

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

		# Create can bus
		self.bus = can.interface.Bus("vcan0")
		self.can_input_thread = threading.Thread(name="can_input", target=self.wait_for_can_message)

		self.subscriber: Subscriber = Subscriber("/can_interface/out", CanData, self.on_ros_message)
		self.publisher: Publisher = Publisher("/can_interface/in", CanData, queue_size=10)

		# Start thread
		self.can_input_thread.start()

	def wait_for_can_message(self):
		'''
			Loop in CAN bus to read data
		'''
		try:
			for message in self.bus:
				try:
					self.on_can_message(message)
				except:
					traceback.print_exc()
					print("received invalid can frame")
					print(message)

				if rospy.is_shutdown():
					self.bus.shutdown()
					return
		except:
			self.bus.shutdown()
			print("Can reception interrupted")
			
	
	def on_can_message(self, frame: can.Message):
		"""
			Callback from messages from can
		"""

		frame_type = self.frames.by_id[frame.data[0]]
		print("received frame {}".format(frame_type.name))
		# Handle pong data
		if frame_type.name == "pong":
			address: int = int(frame.data[1])
			status: int = frame.data[2]

			# Set status if in devices
			if address in self.devices.by_id:
				self.set_node_status(self.devices.by_id[address], "board", NodeStatus.READY)
		else:
			# TODO check if broadcast or to bbb
			values = Argumentable()

			# Add all parameters
			for param in frame_type.params:
				param.can_to_ros(frame, values)

			# Create message
			message = CanData()
			message.params = values.to_list()
			message.type = frame_type.name

			# Publish
			self.publisher.publish(message)
				
	def on_ros_message(self, message):
		"""
			Handle message from ROS and build a frame from it
		"""
		
		rospy.logdebug("received frame to send of type {}".format(message.type))

		# Get message params as argumentable
		values = Argumentable().from_list(message.params)
		frame_type = self.frames.by_name[message.type]

		# Prepare data array and set frame type
		data_array: List[int] = [0] * 8
		data_array[0] = frame_type.id

		# Add parameters provided in message
		try:
			for param in frame_type.params:
				param.ros_to_can(data_array, values)
		except MissingParameterException as e:
			rospy.logerr("unable to find parameter {} for frame {}, not sending"
				.format(e, frame_type.name))
			return
		
		# Setup output frame
		frame: can.Message = can.Message()
		frame.timestamp = time.time()
		frame.is_remote_frame = 0
		frame.is_error_frame = 0
		frame.is_extended_id = 0
		frame.dlc = 1
		frame.arbitration_id = self.devices.by_name[frame_type.dest]
		frame.data = bytes(data_array) # Apply data to frame

		# Send frame to ros
		self.bus.send(frame)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = CanInterfaceNode()
		print("ready")
		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
		
		node.bus.shutdown()
	except rospy.ROSInterruptException:
		pass