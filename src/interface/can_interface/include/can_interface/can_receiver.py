#!/usr/bin/python3
import datetime
import time
import can
can.rc['interface'] = 'socketcan_ctypes'

from can.interface import Bus
from can import Message
 
def check_rx(id,data):
	now = datetime.datetime.now()
	timeString = now.strftime("%d.%m.%Y %H:%M:%S ")
	print(timeString, " ID ", id, " Data", data)


can_interface = 'can1'
bus = Bus(can_interface)

try:
	print("receiver ready")
	for message in bus:
		check_rx(message.arbitration_id, message.data[0])
except KeyboardInterrupt:
	bus.shutdown()