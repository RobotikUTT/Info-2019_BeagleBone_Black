#!/usr/bin/python3
import sys
import unittest

import rospy
import can

from can_interface import FrameList, DeviceList
from args_lib.argumentable import Argumentable

def main():
	devices = DeviceList.parse_file("can/devices.xml", "interface_description")
	frames = FrameList.parse_file("can/frames.xml", "interface_description", context={"devices": devices})
	
	frame_name = input("can frame to send : ")

	while frame_name not in frames.by_name:
		print("available frames :\n\t{}".format("\n\t".join(frames.by_name.keys())))
		frame_name = input("can frame to send : ")
		
	frame = frames.by_name[frame_name]

	values = Argumentable()
	for param in frame.params:
		val = None

		while val is None:
			try:
				val = int(input("value for {} : ".format(param.name)))
			except ValueError:
				print("value must be int")
		values.set(param.name, val)
	
	print("preparing frame")
	message = can.Message(
		data=frame.get_frame_data(values),
		arbitration_id=devices.by_name[frame.dest],
		is_extended_id=False,
		dlc=frame.size()
	)
	
	interface = input("interface [can0] : ")
	if interface == "":
		interface = "can0"
	
	can.rc['interface'] = 'socketcan_ctypes'	
	bus = can.Bus(interface)

	print("sending frame")
	bus.send(message)

if __name__ == '__main__':
	main()