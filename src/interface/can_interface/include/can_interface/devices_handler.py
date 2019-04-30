# File adding an handler for WHOAMI frames and devices
from node_watcher import NodeStatusHandler

from .input import InputElement
from .output import OutputElement
from .io_element import IOElement

from can_msgs.msg import Frame
from ai_msgs.msg import NodeStatus

from typing import Dict, List

from xml_class_parser import Parsable, Bind, BindDict, BindList, Context

@Parsable(
	name="device",
	children = [
		BindList(to="io", type=InputElement),
		BindList(to="io", type=OutputElement)
	]
)
class Device:
	def __init__(self):
		self.id = 0
		self.name = ""
		self.io: List[IOElement] = []

@Parsable(
	name="devices",
	children=[
		BindDict(to="devices", key="id", type=Device),
		Bind(to="broadcast", xml_name="broadcast", type=Device),
		Bind(to="self", xml_name="self", type=Device)
	]
)
class DevicesHandler(NodeStatusHandler):
	def __init__(self, context: Context):
		super().__init__()

		# Devices and their address
		self.names: Dict[int, str] = {}
		self.addresses: Dict[str, int] = {}
		
		self.self: int = -1
		self.broadcast: int = 0
		self.elements: List[IOElement] = []

	def __parsed__(self, context: Context):
		context.get("interface").subscribe(Frame.ORDER_WHOAMI, self)

		# Parsing mapping file
		#root: ElementTree.Element = ElementTree.parse(source_folder + "/can/devices.xml").getroot()


	def on_can_message(self, frame):
		address: int = frame.data[1]
		status: int = frame.data[2]
		
		if address in self.addresses:
			self.set_node_status(self.names[address], "board", NodeStatus.READY)

		
