#!/usr/bin/python3

from action_manager import ActionPerformer
from args_lib.argumentable import Argumentable

from typing import Dict
from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, NodeStatus, ActionStatus

import rospy
import time

class SleepActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("sleep")
		self.set_status(NodeStatus.READY)
	
	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Argumentable):
		time.sleep(args.get("duration", float))

		# Mark action as done after waiting
		self.returns(ActionStatus.DONE)

class FailActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("fail")
		self.set_status(NodeStatus.READY)
	
	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Argumentable):
		# Mark action as paused
		self.returns(ActionStatus.PAUSED)

class MoveActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("move")
		self.set_status(NodeStatus.READY)
	
	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = Pose2D()
		res.end.x = args.get("x", float, 0)
		res.end.y = args.get("y", float, 0)
		res.end.theta = args.get("angle", float, 0)

		return res
	
	def start(self, args: Argumentable):
		# Mark action as paused
		self.returns(ActionStatus.PAUSED)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('test_performers')

	try:
		# Create perfomers
		node = MoveActionPerformer()
		node = FailActionPerformer()
		node = SleepActionPerformer()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass