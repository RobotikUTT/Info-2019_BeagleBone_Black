#!/usr/bin/python3

import can

from can.interface import Bus
from can import Message

can_interface = 'can1'
bus = Bus(can_interface)
print("Send a message...")
msg = Message()
msg.extended_id = True
msg.is_remote_frame = False
msg.id_type = 1
msg.is_error_frame = False
msg.arbitration_id = 0x00E07123
msg.dlc = 1
msg.data = [ 0x01]

bus.send(msg)