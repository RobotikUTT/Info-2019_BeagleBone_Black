from importlib import import_module
import re

from can_msgs.msg import Frame
from rospy import Publisher, Subscriber
from interface_msgs import msg as interface_msgs

class FrameSubscriber:
	def on_can_message(self, frame):
		pass

class CanInterface:
	def __init__(self):
		self.includes = {}
		self.subscribers = {}

		self.can_publisher = Publisher('sent_messages', Frame, queue_size=10)
		self.can_subscriber = Subscriber('received_messages', Frame, self.on_can_message)

	def include(self, message_name, package="interface_msgs"):
		# TODO handle std_msgs
		return getattr(interface_msgs, message_name)
	
	def subscribe(self, frame, subscriber):
		self.subscribers[frame] = subscriber

	def on_can_message(self, frame):
		if frame.id == Frame.BBB_CAN_ADDR:
			if frame.data[0] in self.subscribers:
				self.subscribers[frame.data[0]]
		# TODO handle WHOAMI frames