from rospy import Publisher, Subscriber, Time
from can_msgs.msg import Frame

from interface_description.msg import InterfaceTopics as Topics

# Can library
import can

import time

from typing import Dict, List

class Param:
	def __init__(self, name):
		self.name: str = name
		self.size: int = -1
		self.byte_start: int = 0
	
	def __str__(self):
		return "{}[{}-{}]".format(self.name, self.byte_start, self.byte_start + self.size - 1)

class IOElement:
	def __init__(self, settings, params, interface):
		self.settings: dict = settings
		self.params: List[Param] = []

		# Set settings topic to real topic
		self.settings["topic"] = getattr(Topics, self.settings["topic"])

		# Convert params XML list to param list
		current_offset: int = 1

		for param in params:
			current = Param(
				param.attrib["name"]
			)

			if param.tag == "byte":
				current.size = 1
			elif param.tag == "word":
				current.size = 2
			else:
				print("unknown tag '{}', please use <word> or <byte>".format(param.tag))
			
			# Compute byte_start for each value (offset 0 saved for frame)
			current.byte_start = current_offset
			current_offset += current.size

			self.params.append(current)
		
		# Include required message
		self.message = interface.include(self.settings["msg"])


class InputElement(IOElement):
	def __init__(self, settings, params, interface):
		IOElement.__init__(self, settings, params, interface)

		self.publisher: Publisher = Publisher(self.settings["topic"], self.message, queue_size=10)
		interface.subscribe(self.settings["frame"], self)
	
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
			self.settings["frame"], self.settings["topic"], self.settings["msg"],
			", ".join(map(lambda x: x.__str__(), self.params))
		)


class OutputElement(IOElement):
	def __init__(self, settings, params, id, interface):
		IOElement.__init__(self, settings, params, interface)

		self.device_id = id
		self.subscriber = Subscriber(self.settings["topic"], self.message, self.on_ros_message)

		self.command_type: int = getattr(Frame, self.settings["frame"])

		self.bus = can.interface.Bus("can1")

	def on_ros_message(self, message):
		frame: can.Message = can.Message()

		# Basic settings
		frame.stamp = Time.now()
		frame.is_remote_frame = 0 #
		frame.is_error_frame = 0 #
		frame.extended_id = 0 #
		frame.dlc = 1 #

		frame.arbitration_id = self.device_id

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
			self.settings["frame"], self.settings["topic"], self.settings["msg"],
			", ".join(map(lambda x: x.__str__(), self.params))
		)

