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
		self.set_node_status("can_interface", "interface", NodeStatus.INIT)

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

		# Declare can_interface ready
		self.set_node_status("can_interface", "interface", NodeStatus.READY)

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
		if len(frame.data) == 0:
			rospy.logerr("received empty frame, ignoring")
			return

		elif frame.data[0] not in self.frames.by_id:
			rospy.logerr("received unhandled frame of id {}".format(frame.data[0]))
			return
		
		# Get frame type
		frame_type = self.frames.by_id[frame.data[0]]
		
		rospy.logdebug("received frame from can of type {}".format(frame_type.name))

		# Handle pong data
		if frame_type.name == "pong":
			if len(frame.data) < 3:
				rospy.logerr("received incomplete pong frame, ignoring".format(frame_type.name))

			address: int = int(frame.data[1])
			status: int = frame.data[2]

			# Set status if in devices
			if address in self.devices.by_id:
				self.set_node_status(self.devices.by_id[address], "board", NodeStatus.READY)
			else:
				rospy.logerr("received pong frame for an unknown device of id {}".format(address))
		else:
			# TODO check if broadcast or to bbb
			# Create message
			message = CanData()
			try:
				message.params = frame_type.extract_frame_data(frame).to_list()
			except IndexError:
				rospy.logerr("received incomplete frame of type {}, ignoring".format(frame_type.name))
				return

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
		data = frame_type.get_frame_data(values)
		
		if data is not None:
			# Setup output frame
			frame: can.Message = can.Message(
				timestamp=time.time(),
				is_remote_frame=False,
				is_error_frame=False,
				is_extended_id=False,
				dlc=frame_type.size(),
				arbitration_id=self.devices.by_name[frame_type.dest],
				data=data
			)

			# Send frame to ros
			self.bus.send(frame)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('can_interface')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = CanInterfaceNode()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
		
		node.bus.shutdown()
	except rospy.ROSInterruptException:
		pass