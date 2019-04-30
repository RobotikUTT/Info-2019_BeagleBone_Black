from can_msgs.msg import Frame
from interface_msgs import msg as interface_msgs

from interface_description.msg import InterfaceTopics as Topics

from typing import Dict, List, Union, Callable
from xml_class_parser import Parsable, Bind, BindList, Enum

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
	name="_",
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
		self.message: Callable = lambda: None
		self.topic: str = ""
		
	def __parsed__(self):
		current_offset: int = 1

		# Compute param start
		for param in self.params:
			param.byte_start = current_offset
			current_offset += param.size
		



