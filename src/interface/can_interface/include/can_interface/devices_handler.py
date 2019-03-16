# File adding an handler for WHOAMI frames
from node_watcher.node_status_handler import NodeStatusHandler
from can_msgs import Frame
from ai_msgs import NodeStatus

class DevicesHandler(NodeStatusHandler):
	def __init__(self, interface):
        super().__init__()

		interface.subscribe(Frame.ORDER_WHOAMI, self)

        # Devices and their address
        # TODO load from XML
        self.devices = {
            "STM": 2
        }

    def get_device_address(self, device):
        return self.devices[device]

	def on_can_message(self, frame):
        address: int = msg.data[1]
        status: int = msg.data[2]
        
        name = ""
        # TODO get from devices.xml
        if address == Frame.STM_CAN_ADDR:
            name = "STM"
        elif address == Frame.ARDUINO_CAN_ADDR:
            name = "ARDUINO"
        elif address == Frame.ZIGBEE_CAN_ADDR:
            name = "ZIGBEE"
        elif address == Frame.PANEL_CAN_ADDR:
            name = "PANEL"

		self.set_node_status(name, "board", NodeStatus.READY)
