from action_manager import ActionPerformer

from typing import Dict
from geometry_msgs.msg import Pose2D
from ai_msgs.msg import ActionPoint

import rospy
import time

class SleepActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("sleep")
	
	def compute_action_point(self, args: Dict[str, float], robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Dict[str, str]):
		time.sleep(self.get_int("duration"))

		# Mark action as done after waiting
		self.action_performed()

class FailActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("fail")
	
	def compute_action_point(self, args: Dict[str, float], robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = robot_pos
		return res
	
	def start(self, args: Dict[str, float]):
		# Mark action as paused
		self.action_paused()

class MoveActionPerformer(ActionPerformer):
	def __init__(self):
		super().__init__("move")
	
	def compute_action_point(self, args: Dict[str, float], robot_pos: Pose2D) -> ActionPoint:
		res = ActionPoint()
		res.start = robot_pos
		res.end = Pose2D()
		res.end.x = self.get_float("x", 0)
		res.end.y = self.get_float("y", 0)
		res.end.theta = self.get_float("angle", 0)

		return res
	
	def start(self, args: Dict[str, str]):
		# Mark action as paused
		self.action_paused()



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