# File adding an handler for WHOAMI frames and devices
from node_watcher.node_status_handler import NodeStatusHandler

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

		# Find mapping file directory
		source_folder = rospkg.RosPack().get_path("interface_description")

		# Parsing mapping file
		root = ElementTree.parse(source_folder + "/can/devices.xml").getroot()

		if root.tag != "devices":
			print("invalid file, must contains a <devices> root element")
			exit(0)

		for child in root:
			id: int = int(child.attrib["id"])

			if child.tag == "self":
				self.self = int(child.attrib["id"])
			elif child.tag == "broadcast":
				self.broadcast = int(child.attrib["id"])
				self.addresses["broadcast"] = id
			elif child.tag == "device":
				name: str = child.attrib["name"]

				self.addresses[name] = id
				self.names[id] = name
			else:
				print("unknow element :", child.tag)

	def on_can_message(self, frame):
		address: int = frame.data[1]
		status: int = frame.data[2]
		
		if address in self.addresses:
			self.set_node_status(self.names[address], "board", NodeStatus.READY)

		
