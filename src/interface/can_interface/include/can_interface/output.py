from rospy import Subscriber, Time
from can_msgs.msg import Frame

from .io_element import IOElement, Param

import can # Can library

import time

from typing import Dict, List, Union, Callable
from xml_class_parser import Parsable, Bind, BindDict, BindList, Enum, Context

@Parsable(
	name = "output",
	extends = IOElement
)
class OutputElement(IOElement):
	def __parsed__(self, context: Context):
		self.device: 'Device' = context.parent # parent is supposed to be a device
		self.interface = context.get("interface")
		self.subscriber = Subscriber(self.topic, self.message, self.on_ros_message)

		self.command_type: int = getattr(Frame, self.frame)
		self.bus = can.interface.Bus("can1")

	def on_ros_message(self, message):
		frame: can.Message = can.Message()

		# Basic settings
		frame.stamp = time.time()
		frame.is_remote_frame = 0
		frame.is_error_frame = 0
		frame.extended_id = 0
		frame.dlc = 1

		frame.arbitration_id = self.device.id

		data_array: List[int] = [0] * 8

		# Add params
		for param in self.params:
			self.put_param_value(data_array, message, param)
		
		# Set frame type
		data_array[0] = self.command_type

		frame.data = bytes(data_array)

		# Send frame to ros
		self.bus.send(frame)

	def put_param_value(self, data_array: List[int], msg, param: Param):
		if param.size == 1:
			data_array[param.byte_start] = getattr(msg, param.name)
		elif param.size == 2:
			data_array[param.byte_start] = getattr(msg, param.name) >> 8
			data_array[param.byte_start + 1] = getattr(msg, param.name) & 0x00FF
		else:
			print("size not handled yet, go back to coding")
	
	def __str__(self):
		return "Output[{}, {}, {}] : {}".format(
			self.frame, self.topic, self.message,
			", ".join(map(lambda x: x.__str__(), self.params))
		)

