import rospy
import rospkg

from node_status_handler import NodeStatusHandler

from ai_msgs.msg import AwaitNodesResult, Topics, NodeStatus
from std_msgs.msg import Int32

from abc import ABC, abstractmethod

class Node (NodeStatusHandler, ABC):
'''
	Class describing a generic ROS node, registered to
	the node_watcher_node.
'''

	def __init__(nodename: str, package: str):
		super().__init__()

		self.nodename = nodename
		self.nodepath = self._make_node_path(nodename, package)

		self._status = NodeStatus()
		self.set_node_status(NodeStatus.NODE_INIT)

		self.set_wait_callback(self.on_waiting_result)

	def _on_await_response(self, msg: AwaitNodesResult):
		if msg.request_code == self._wait_request_code:
			rospy.log("Node {} is done waiting for nodes".format(self.nodename))
			self._wait_callback(msg.success)

	@abstractmethod
	def on_waiting_result(success: bool):
		pass 

	# Status setter
	def set_status(state_code: int, error_code: int = 0):
		# Save status
		self._status.state_code = state_code
		self._status.error_code = error_code

		# Update remotely
		super().set_node_status(
			self.nodename, self.nodepath, state_code, error_code
		)

	# add self status getter
	def get_status(self, remote: bool = False):
		if remote:
			return super().get_node_status(self.nodename, self.nodepath)
		else:
			return self._status

	def wait_for_nodes(self, timeout: int, use_pkg_file: bool = False):
		# If we use the package "requirements.txt" file
		if use_pkg_file:
			# We add requirements to it
			filename = rospkg.RosPack().get_path(self.nodename) + "/requirements.txt"
			self.require_from_file(filename)
		
		# Otherwise just wait
		super().wait_for_nodes(timeout)

