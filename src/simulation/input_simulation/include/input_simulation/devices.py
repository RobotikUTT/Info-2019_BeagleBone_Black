# File adding an handler for WHOAMI frames and devices
import rospkg

from can_msgs.msg import Frame
from ai_msgs.msg import NodeStatus

from xml.etree import ElementTree

from typing import List

DEVICES = {}

def parse_frames():
	# Parsing mapping file
	root = ElementTree.parse(
		rospkg.RosPack().get_path("interface_description") +
		"/can/devices.xml"
	).getroot()

	# Parse various devices
	for child in root:
		id: int = int(child.attrib["id"])
		name: str = ""

		# Save id in appropriate
		if child.tag == "self" or child.tag == "broadcast":
			name = child.tag
		elif child.tag == "device":
			name = child.attrib["name"]
		else:
			continue
		
		devices[name] = Device(id, name, child)

class Param():
	def __init__(self, param_object):
		self.size = 2 if param_object.tag == "word" else 1
		self.offset = -1

	def apply(self, data_array, value):
		'''Set value in the given byte array'''
		if self.size == 1:
			data_array[self.offset] = value
		elif self.size == 2:
			data_array[self.offset] = value >> 8
			data_array[self.offset + 1] = value & 0x00FF
		
	def get(self, frame):
		'''Get value in the frame data'''
		value = frame.data[self.offset + self.size - 1]

		for index in range(self.size - 1):
			value = value | \
				frame.data[self.offset + index] << (self.size - index - 1) * 8

		return value

class Device():
	def __init__(self, id: int, name: str, iolist):
		self.name = name
		self.id = id
		self.inputs = {}
		self.outputs = {}

		# For each compatible device, parse it's inputs and outputs
		for child in iolist:
			params = {}
			offset = 0

			for param in child:
				obj = Param(param)
				obj.offset = offset

				offset += obj.size
				params[param.attrib["name"]] = obj

			if child.tag == "input":
				self.inputs[child.attrib["frame"]] = params
			elif child.tag == "output":
				self.outputs[child.attrib["frame"]] = params
		
