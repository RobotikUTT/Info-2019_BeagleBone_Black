import rospy
import math

from ai_msgs.msg import NodeStatus
from interface_msgs.msg import StmMode

from geometry_msgs.msg import Pose2D

from typing import List

import time

# Consts for goals
ORIENTED_POSITION = 2
POSITION = 1
ORIENTATION = 0

class FakeStm:
	def __init__(self, master_node):
		self.goals = []
		self.pose = Pose2D()
		self.pose_offset = Pose2D()
		self.mode: int = StmMode.STOP
		self.goal = None

		self.done = False
		self.master_node = master_node

	def can_data(self, name, params, node):
		if name == "set_position":
			pass
		elif name == "set_stm_mode":
			pass

		elif name == "go_to":
			self.done = time.time()
		elif name == "go_to_angle":
			self.done = time.time()
		elif name == "rotate":
			self.done = time.time()
	
	def spin(self):
		if self.done != False:
			if time.time() - self.done > 1:
				self.done = False
				self.master_node.send_can("order_complete", {})
		

def register(master_node):
	'''Function defining a simulation extentions'''
	stm = FakeStm(master_node)
	master_node.can_subscribers.append(stm)
	master_node.spin_callbacks.append(stm.spin)