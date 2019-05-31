#!/usr/bin/python3
# Bridge interface_msgs to ROS frames as described in interface_description
import rospy

from xml_class_parser import Parsable, Bind, Context

from args_lib.argumentable import Argumentable

from can_interface import Frame, FrameList, DeviceList, Param
from can_interface.param import MissingParameterException

from typing import Type, Dict, List

import time
import threading
import traceback

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

class CanHandler:

	def __init__(self, dev="vcan0", callback=None):
		self.callback = callback

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

		# Create can bus with given interface
		self.bus = can.interface.Bus(dev)
		self.can_input_thread = threading.Thread(name="can_input", target=self.wait_for_can_message)

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
					rospy.logerr("received invalid can frame")
					rospy.logerr(message)

				if rospy.is_shutdown():
					self.bus.shutdown()
					return
		except:
			self.bus.shutdown()
			rospy.logwarn("Can reception interrupted")
			
	
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

		# Create message
		try:
			params = frame_type.extract_frame_data(frame).to_list()
		except IndexError:
			rospy.logerr("received incomplete frame of type {}, ignoring".format(frame_type.name))
			return

		if self.callback is not None:
			self.callback(frame_type.name, params)

	def send(self, type, params: Argumentable):
		"""
			Handle message from ROS and build a frame from it
		"""
		
		rospy.logdebug("received frame to send of type {}".format(message.type))

		# Get message params as argumentable
		values = params
		frame_type = self.frames.by_name[type]

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

