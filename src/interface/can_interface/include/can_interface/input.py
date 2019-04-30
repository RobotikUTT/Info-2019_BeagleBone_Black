from rospy import Publisher
from can_msgs.msg import Frame

from .io_element import IOElement, Param

import can # Can library

from typing import Dict, List, Union, Callable
from xml_class_parser import Parsable, Context

@Parsable(
	name = "input",
	extends = IOElement
)
class InputElement(IOElement):
	def __parsed__(self, context: Context):
		context.get("interface").subscribe(self.frame, self)
		
		self.publisher: Publisher = Publisher(self.topic, self.message, queue_size=10)
	
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