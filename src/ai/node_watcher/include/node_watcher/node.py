import rospy
import rospkg

from node_watcher.node_status_handler import NodeStatusHandler

from ai_msgs.msg import AwaitNodesResult, Topics, NodeStatus
from std_msgs.msg import Int32

from abc import ABC, abstractmethod

class Node (NodeStatusHandler, ABC):
	'''
		Class describing a generic ROS node, registered to
		the node_watcher_node.
	'''

	def __init__(self, nodename: str, package: str):
		super().__init__()

		self.nodename = nodename
		self.package = package

		self._status = NodeStatus()
		self.set_status(NodeStatus.INIT)

		self.set_wait_callback(self.on_waiting_result)

	def _on_await_response(self, msg: AwaitNodesResult):
		if msg.request_code == self._wait_request_code:
			rospy.loginfo("Node {} is done waiting for nodes".format(self.nodename))
			self._wait_callback(msg.success)

	def on_waiting_result(self, success: bool):
		pass 

	# Status setter
	def set_status(self, state_code: int, error_code: int = 0):
		# Save status
		self._status.state_code = state_code
		self._status.error_code = error_code

		# Update remotely
		super().set_node_status(
			self.nodename, self.package, state_code, error_code
		)

	# add self status getter
	def get_status(self, remote: bool = False):
		if remote:
			return super().get_node_status(self.nodename, self.package)
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

