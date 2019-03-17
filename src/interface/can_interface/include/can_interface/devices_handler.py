# File adding an handler for WHOAMI frames and devices
from node_watcher.node_status_handler import NodeStatusHandler

from can_interface.io_elements import InputElement, OutputElement

from can_msgs import Frame
from ai_msgs import NodeStatus

import rospkg

from xml.etree import ElementTree

class DevicesHandler(NodeStatusHandler):
	def __init__(self, interface):
		super().__init__()

		interface.subscribe(Frame.ORDER_WHOAMI, self)

		# Devices and their address
		self.devices = {}
		self.addresses = {}
		
		self.self = -1
		self.broadcast = 0
		self.elements = []

		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("interface_description")

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/can/devices.xml").getroot()

		if root.tag != "devices":
			print("invalid file, must contains a <devices> root element")
			exit(0)

		# Parse various devices
		for child in root:
			id: int = int(child.attrib["id"])

			# Save id in appropriate
			if child.tag == "self":
				self.self = id
			elif child.tag == "broadcast":
				self.broadcast = id
				self.addresses["broadcast"] = id
			elif child.tag == "device":
				name: str = child.attrib["name"]

				self.addresses[name] = id
				self.names[id] = name
			else:
				print("unknow element :", child.tag)
				continue
			
			# For each compatible device, parse it's inputs and outputs
			for iochild in child:
				if iochild.tag == "input":
					self.elements.append(InputElement(iochild.attrib, iochild, interface))
				elif iochild.tag == "output":
					self.elements.append(OutputElement(iochild.attrib, iochild, id, interface))
				else:
					print("unknow element :", iochild.tag)

	def on_can_message(self, frame):
		address: int = frame.data[1]
		status: int = frame.data[2]
		
		if address in self.addresses:
			self.set_node_status(self.names[address], "board", NodeStatus.READY)

		
