#!/usr/bin/python3

from action_manager import ActionPerformer
from args_lib.argumentable import Argumentable

from typing import Dict
from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, NodeStatus, ActionStatus

import rospy

class TemplateActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("template")
		self.set_status(NodeStatus.READY)
	
	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		# Compute start and end of action
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Argumentable):
		# [[ ACTION CODE HERE ]]

		# Mark action as done after completion
		self.returns(ActionStatus.DONE)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('template')

	try:
		# Init perfomers
		node = TemplateActionPerformer()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass