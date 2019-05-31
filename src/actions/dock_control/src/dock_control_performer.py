#!/usr/bin/python3

from action_manager import ActionPerformer
from args_lib.argumentable import Argumentable

from typing import Dict
from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint, NodeStatus, ActionStatus

from interface_msgs.msg import CanData

import rospy

class DockControlActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("dock_control")
		self.set_status(NodeStatus.READY)
		
		self.can_in = rospy.Subscriber("/can_interface/in", CanData, self.on_can)
		self.can_out = rospy.Publisher("/can_interface/out", CanData, queue_size=10)
	
	def on_can(self, frame):
		if frame.type == "your_dock_has_fullfilled_your_request":
			# It's ok !
			self.returns(ActionStatus.DONE)

	def compute_action_point(self, args: Argumentable, robot_pos: Pose2D) -> ActionPoint:
		# Compute start and end of action
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Argumentable):
		msg = CanData()
		msg.type = "set_dock_height"
		msg.params = args.to_list()
		self.can_out.publish(msg)


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('dock_control')

	try:
		# Init perfomers
		node = DockControlActionPerformer()

		# Spin
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass