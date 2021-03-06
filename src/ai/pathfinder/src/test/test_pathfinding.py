#!/usr/bin/python3
import sys
import unittest

from node_watcher import NodeStatusHandler

from ai_msgs.msg import NodeStatus
from geometry_msgs.msg import Pose2D

from pathfinder.srv import FindPath, SetMapDimension

import time

import rospy
import rostest

class TestPathfinding(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_pathfinding', anonymous=True)

		# Declare no shape and set map_handler node ready -> should make pathfinder ready
		self.ns_handler = NodeStatusHandler()

		self.ns_handler.set_node_status("map_handler", "ai", NodeStatus.READY)

		# Wait for it to be up
		start = time.time()
		while time.time() - start < 1.1 and self.ns_handler.get_node_status("pathfinder", "ai").state_code != NodeStatus.READY:
			time.sleep(0.1)
		
		self.assertLess(time.time() - start, 1, "node wakes up")
		
		set_map_dim = rospy.ServiceProxy("/ai/pathfinder/set_map_dimension")
		set_map_dim.wait_for_service(2)
		
		set_map_dim(200, 100)

	def test_findpath(self):
		findpath = rospy.ServiceProxy("/ai/pathfinder/findpath", FindPath)
		findpath.wait_for_service(2)

		start = Pose2D()
		start.x = 0
		start.y = 0

		end = Pose2D()
		end.x = 200
		end.y = 100

		response = findpath(start, end)

if __name__ == '__main__':
	rostest.rosrun('pathfinder', 'test_pathfinding', TestPathfinding, sys.argv)
