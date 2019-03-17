#!/usr/bin/python3
import rospy

from node_watcher.node_status_handler import NodeStatusHandler
from ai_msgs import NodeStatus, Topics, StartRobot, Side

from input_simulation import modules as simulation_parts

class InputSimulationNode():
	def __init__(self):
		self.nodes = NodeStatusHandler()
		self.start_signal_pub = rospy.Publisher(Topics.START_SIGNAL_TOPIC, StartRobot)
		self.spin_callbacks = []

		self.nodes.require("/ai/controller")
		self.nodes.require("/ai/scheduler")

		self.nodes.set_await_callback(self.nodes_ready)
		self.nodes.wait_for_nodes(6)

	def nodes_ready(self, success: bool):
		if not success:
			rospy.logerr("Controller or scheduler did not woke up...")
			return
		
		msg = StartRobot()
		msg.side = Side.LEFT
		self.start_signal_pub.publish(msg)

	def spin(self):
		while not rospy.is_shutdown():
			# Call registered callbacks
			for callback in self.spin_callbacks:
				callback()

			# Spin with ros
			rospy.spin()

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('input_simulation_node')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		# Create node
		node = InputSimulationNode()
		print(simulation_parts)
		for part in simulation_parts:
			if hasattr(part, "register"):
				part.register(node)

		node.spin()
	except rospy.ROSInterruptException:
		pass
