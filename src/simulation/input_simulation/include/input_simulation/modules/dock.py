import rospy
import math

from ai_msgs.msg import NodeStatus

from geometry_msgs.msg import Pose2D

from typing import List

import time

class Dock:
	def __init__(self, master_node):
		self.done = False
		self.master_node = master_node

	def can_data(self, name, params, node):
		if name == "set_dock_height":
			self.done = time.time()
	
	def spin(self):
		if self.done != False:
			if time.time() - self.done > 0.1:
				self.done = False
				self.master_node.send_can("your_dock_has_fullfilled_your_request", {})
		

def register(master_node):
	'''Function defining a simulation extentions'''
	dock = Dock(master_node)
	master_node.can_subscribers.append(dock)
	master_node.spin_callbacks.append(dock.spin)