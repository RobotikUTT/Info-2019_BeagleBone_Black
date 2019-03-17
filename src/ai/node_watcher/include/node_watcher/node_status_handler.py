import rospy
import rospkg

from ai_msgs.msg import AwaitNodesResult, \
	NodeStatus, NodeRequirement, Topics
from ai_msgs.srv import NodeReadiness, NodeReadinessRequest, AwaitNodesRequest

from std_msgs.msg import Int32

from typing import List, Callable

WaitCallback = Callable[[bool], None]

class NodeStatusHandler:
	'''Class handling any "abstract nodes" (as they are not defined as a single class)'''

	def __init__(self):
		self._watcher_client = rospy.ServiceProxy(Topics.NODE_WATCHER_SERVICE, NodeReadiness)
		self._waiter_client = rospy.ServiceProxy(Topics.NODES_AWAITER_INIT_SERVICE, AwaitNodesRequest)
		self._start_wait_pub = rospy.Publisher(Topics.NODES_AWAITER_START_TOPIC, Int32, queue_size=10)
		self._wait_answer_sub = rospy.Subscriber(Topics.NODES_AWAITER_RESULT_TOPIC, AwaitNodesResult, self._on_await_response)
	
		self._wait_callback: WaitCallback = None
		self._wait_request_code: int = -1

		self._requirements: List[NodeRequirement] = []

		# Wait for service to send init signal
		rospy.wait_for_service(Topics.NODE_WATCHER_SERVICE)

	# Build node path
	def _make_node_path(self, nodename: str, package: str) -> str:
		return "/{}/{}".format(package, nodename)

	# Status getter
	def get_node_status(self, nodename: str, package: str = None) -> NodeStatus:
		if package == None:
			nodename = self._make_node_path(nodename, package)

		msg = NodeReadinessRequest()
		msg.status.state_code = NodeStatus.NODE_ASKING # ask for status
		msg.node_name = nodename

		try:
			response = self._watcher_client(msg)
			return response.status

		except rospy.ServiceException:
			rospy.logerr("Unable to call watcher as getter for {}".format(nodename))
	
	# Status setter
	def set_node_status(self, nodename: str, package: str, state_code: int, error_code: int = 0):
		nodepath = self._make_node_path(nodename, package)

		# update remote version
		msg = NodeReadinessRequest()
		msg.status.state_code = state_code
		msg.status.error_code = error_code
		msg.node_name = nodepath

		try:
			self._watcher_client(msg)
		except rospy.ServiceException:
			rospy.logerr("Unable to call watcher as setter for {}".format(nodepath))
		

	# Node waiting
	def wait_for_nodes(self, timeout: int):
		if len(self._requirements) == 0:
			self._wait_callback(True)
		
		# Call service to retrieve code
		try:
			response = self._waiter_client(self._requirements, timeout)
			self._wait_request_code = response.request_code

			# Wait for publisher to be ready to address not sent issue
			# From [https:#answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/]
			poll_rate = rospy.Rate(100)
			while self._start_wait_pub.get_num_connections() == 0:
				poll_rate.sleep()
			
			# Start waiting
			msg = Int32()
			msg.data = self._wait_request_code
			self._start_wait_pub.publish(msg)
		except rospy.ServiceException:
			rospy.logerr("Unabled to call nodes waiting service")
			self._wait_callback(False)

	# Callback setter
	def set_wait_callback(self, cb: WaitCallback):
		self._wait_callback = cb

	# Add node requirement
	def require_from_file(self, filename: str):
		try:
			file = open(filename, "r")

			for line in file:
				nodename, status = line.split(" ", 2)

				# Invalid status
				if status != "optional" and status != "required":
					rospy.logerr(
						"parsing requirements from {} : {} is not a valid optional argument (optional or required)"
							.format(filename, status)
					)
				else:
					# Save as required
					self.require(nodename, status == "required")
		except:
			rospy.logerr("unable to parse {}".format(filename))

		
	def require(self, nodename: str, package: str, required: bool = True):
		nodepath = self._make_node_path(nodename, package)

		req = NodeRequirement()
		req.nodename = nodepath
		req.optional = not required

		self._requirements.append(req)

	# Requirement getter
	def is_required(self, nodepath: str) -> bool:
		for next in self._requirements:
			if next.nodename == nodepath:
				return True

		return False

	def _on_await_response(self, msg: AwaitNodesResult):
		if msg.request_code == self._wait_request_code:
			self._wait_callback(msg.success)
