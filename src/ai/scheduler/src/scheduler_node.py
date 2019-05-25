#!/usr/bin/python3

import rospy
from rospkg import RosPack

from action_manager import PerformClient, ActionGroup, Action

from interface_msgs.msg import CanData
from ai_msgs.msg import Side, NodeStatus, ActionStatus
from ai_msgs.srv import SetSchedulerState, SetSchedulerStateRequest, SetSchedulerStateResponse
from geometry_msgs.msg import Pose2D

from xml_class_parser import ParsingException

class SchedulerNode(PerformClient):
	def __init__(self):
		super().__init__("scheduler", "ai")

		self.running = False
		self.current_action: Action = None
		self.robot_pos = Pose2D()

		# ROS services / subscribers
		self.control_srv = rospy.Service("/scheduler/do", SetSchedulerState, self.set_state)
		self.robot_pos_sub = rospy.Subscriber("/can_interface/in", CanData, self.on_can_message)

		# Parse actions
		try:
			folder = RosPack().get_path("ai_description") + "/actions/"
			self.root_action = ActionGroup.parse_file(
				folder + "main.xml", context={"folder": folder})
		except ParsingException as e:
			self.set_status(NodeStatus.ERROR, 1)
			raise e

		# Load requirements
		self.save_required(self.root_action)
		self.wait_for_nodes(5)

	def on_waiting_result(self, success: bool):
		if success:
			self.set_status(NodeStatus.READY)
		else:
			rospy.logerr("Some actions are missing, unable to start node")
			self.set_status(NodeStatus.ERROR, 2)

	def set_state(self, request: SetSchedulerStateRequest):
		"""
			Service function to change scheduler. It change state only if
			requested state is different
		"""
		# If there is a change in state
		if self.running != request.running:
			self.running = request.running
			
			# apply change
			if request.running:
				rospy.loginfo("Scheduler actions resumed !")
				
				# Set side
				self.root_action.set_side(request.side)

				# Resume action
				self.next_action()
			else:
				rospy.loginfo("Scheduler actions paused...")
				self.cancel_action()

		return SetSchedulerStateResponse()

	def on_can_message(self, data: CanData):
		"""
			Handle robot position from STM
		"""
		# Right type of data
		if data.type == "robot_position":
			self.robot_pos.x = data.params.get("x", int)
			self.robot_pos.y = data.params.get("y", int)
			self.robot_pos.theta = data.params.get("theta", int)

	def on_action_returns(self, state, result):
		"""
			Current action done
		"""
		self.current_action.state = result.state
		self.next_action()

	def next_action(self):
		if not self.running:
			return
		
		# Following peace of code assert once a group is launched, it's actions
		# are performing until group state is not IDLE
		current_root = self.root_action
		if self.current_action is not None:
			# Compute local root to continue previously selected group until it is done
			current_root = self.current_action

			while current_root.state != ActionStatus.IDLE and \
				current_root.parent is not None:

				current_root = current_root.parent


		# If pause cancer has spread into the root of the action tree
		if self.root_action.state == ActionStatus.PAUSED:
			# We cure it
			self.root_action.state = ActionStatus.IDLE
		
		rospy.loginfo("[Current action tree]\n{}".format(self.root_action))

		# If there is alive actions again
		if self.root_action.state == ActionStatus.IDLE:
			choice: ActionChoice = self.root_action.get_optimal(self.robot_pos)


			if choice.action is not None:
				# save current action
				self.current_action = choice.action

				rospy.loginfo("Perfoming : {}".format(self.current_action))

				# call then onfinished or onpause
				self.perform_action(choice.action, self.robot_pos)
			else:
				rospy.loginfo("No action to be performed")
				self.root_action.state = ActionStatus.DONE
			
		else:
			# Otherwise signal that everything is over :)
			rospy.loginfo("Root action is in state {}, nothing more to be done...".format(self.root_action.state))
		
if __name__ == "__main__":
	rospy.init_node("scheduler_node", log_level=rospy.INFO)
	node = SchedulerNode()

	while not rospy.is_shutdown():
		rospy.spin()