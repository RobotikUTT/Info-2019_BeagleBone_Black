from rospy import Publisher, Subscriber, Time
from can_msgs.msg import Frame
from interface_msgs import msg as interface_msgs

from interface_description.msg import InterfaceTopics as Topics

from .devices_handler import Device

# Can library
import can

import time

from typing import Dict, List, Union, Callable
from xml_class_parser import Parsable, Bind, BindDict, BindList, Enum

@Parsable(
	name = Bind(to="size", type=Enum(binding={ "word": 2, "byte": 1 })),
	attributes = {
		"name": Bind(mandatory=True)
	}
)
class Param:
	def __init__(self):
		self.name: str = ""
		self.size: int = -1
		self.byte_start: int = 0

	def __str__(self):
		return "{}[{}-{}]".format(self.name, self.byte_start, self.byte_start + self.size - 1)

@Parsable(
	attributes = {
		"frame": Bind(mandatory=True),
		"message": Bind(mandatory=True, type=lambda n: getattr(interface_msgs, n)),
		"topic": Bind(mandatory=True, type=Enum(binding=Topics))
	},
	children = [
		BindList(to="params", type=Param)
	]
)
class IOElement:
	def __init__(self):
		self.params: List[Param] = []
		self.frame: str = ""
		self.message: Callable = None
		self.topic: str = ""
		
	def __parsed__(self):
		current_offset: int = 1

		# Compute param start
		for param in self.params:
			param.byte_start = current_offset
			current_offset += param.size
		

@Parsable(
	name = "input",
	extends = IOElement
)
class InputElement(IOElement):
	def __init__(self, interface):
		super().__init__()
		self.interface = interface

	def __parsed__(self, interface):
		self.publisher: Publisher = Publisher(self.topic, self.message, queue_size=10)
		self.interface.subscribe(self.frame, self)
	
	def on_can_message(self, frame):
		# Create message
		message = self.message()

		# Add all parameters
		for param in self.params:
			setattr(message, param.name, self.get_param_value(frame, param))
		
		# Publish
		self.publisher.publish(message)


	# Retrieve param data from frame data with binary operations
	def get_param_value(self, frame: can.Message, param: Param):
		value = frame.data[param.byte_start + param.size - 1]

		for index in range(param.size - 1):
			value = value | \
				frame.data[param.byte_start + index] << (param.size - index - 1) * 8

		return value
	
	def __str__(self):
		return "Input[{}, {}, {}] : {}".format(
			self.frame, self.topic, self.message,
			", ".join(map(lambda x: x.__str__(), self.params))
		)

@Parsable(
	name = "output",
	extends = IOElement
)
class OutputElement(IOElement):
	def __parsed__(self, device: Device, interface):
		self.device: Device = device
		self.subscriber = Subscriber(self.topic, self.message, self.on_ros_message)

		self.command_type: int = getattr(Frame, self.frame)
		self.bus = can.interface.Bus("can1")

	def on_ros_message(self, message):
		frame: can.Message = can.Message()

		# Basic settings
		frame.stamp = Time.now()
		frame.is_remote_frame = 0 #
		frame.is_error_frame = 0 #
		frame.extended_id = 0 #
		frame.dlc = 1 #

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

