#!/usr/bin/python3
import rospy

from node_watcher import NodeStatusHandler
from ai_msgs.msg import NodeStatus, Topics, StartRobot, Side

from input_simulation.modules import modules as simulation_parts

from xml_class_parser import Context

from args_lib.argumentable import Argumentable

from can_interface import Frame, FrameList, DeviceList, Param
from can_interface.param import MissingParameterException

from typing import Type, Dict, List

import time
import threading
import traceback

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

class RobotSimulationNode():
	def __init__(self):
		self.nodes = NodeStatusHandler()
		self.start_signal_pub = rospy.Publisher(Topics.START_SIGNAL_TOPIC, StartRobot, queue_size=10)
		self.spin_callbacks = []
		self.can_subscribers = []

		# Parse devices
		self.devices: DeviceList = DeviceList.parse_file(
			"can/devices.xml",
			package="interface_description"
		)

		# Then frames
		self.frames: Dict[str, Frame] = FrameList.parse_file(
			"can/frames.xml",
			package="interface_description",
			context=Context(devices=self.devices)
		)

		# Create can bus with given interface
		self.bus = can.interface.Bus(rospy.get_param("/interface/can_interface/device", default="vcan0"))
		self.can_input_thread = threading.Thread(name="can_input", target=self.wait_for_can_message)

		# Start thread
		self.can_input_thread.start()

		# Wait for controller and scheduler
		self.nodes.require("controller", "ai")
		self.nodes.require("scheduler", "ai")
		self.nodes.set_wait_callback(self.nodes_ready)
		self.nodes.wait_for_nodes(6)

	def nodes_ready(self, success: bool):
		if not success:
			rospy.logerr("Controller or scheduler did not woke up...")
			return

		# Wait for a connection
		poll_rate = rospy.Rate(100)
		while self.start_signal_pub.get_num_connections() == 0:
			poll_rate.sleep()

		msg = StartRobot()
		msg.side = rospy.get_param("/simulation/side", Side.DOWN)
		self.start_signal_pub.publish(msg)
		rospy.loginfo("Start signal sent")


	def spin(self):
		poll_rate = rospy.Rate(300)

		while not rospy.is_shutdown():
			# Call registered callbacks

			for callback in self.spin_callbacks:
				callback()

			# Spin with ros
			poll_rate.sleep()
		
		self.bus.shutdown()
		

	def wait_for_can_message(self):
		'''
			Loop in CAN bus to read data
		'''
		try:
			for message in self.bus:
				try:
					self.on_can_message(message)
				except:
					traceback.print_exc()
					rospy.logerr("received invalid can frame")
					rospy.logerr(message)

				if rospy.is_shutdown():
					self.bus.shutdown()
					return
		except:
			self.bus.shutdown()
			rospy.logwarn("Can reception interrupted")
			
	
	def on_can_message(self, frame: can.Message):
		"""
			Callback from messages from can
		"""
		if len(frame.data) == 0:
			rospy.logerr("received empty frame, ignoring")
			return

		elif frame.data[0] not in self.frames.by_id:
			rospy.logerr("received unhandled frame of id {}".format(frame.data[0]))
			return
		
		# Get frame type
		frame_type = self.frames.by_id[frame.data[0]]
		
		# Handle frames
		for handler in self.can_subscribers:
			handler.can_data(frame_type.name, frame_type.extract_frame_data(frame), self)

				
	def send_can(self, type, params: Dict):
		"""
			Handle message from ROS and build a frame from it
		"""
		
		# Get message params as argumentable
		values = Argumentable(params)
		frame_type = self.frames.by_name[type]

		# Prepare data array and set frame type
		data = frame_type.get_frame_data(values)
		
		if data is not None:
			# Setup output frame
			frame: can.Message = can.Message(
				timestamp=time.time(),
				is_remote_frame=False,
				is_error_frame=False,
				is_extended_id=False,
				dlc=frame_type.size(),
				arbitration_id=self.devices.by_name[frame_type.dest],
				data=data
			)

			# Send frame to ros
			self.bus.send(frame)



if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('input_simulation_node')
	
	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = RobotSimulationNode()
		
		for part in simulation_parts:
			if hasattr(part, "register"):
				part.register(node)

		node.spin()
	except rospy.ROSInterruptException:
		pass

