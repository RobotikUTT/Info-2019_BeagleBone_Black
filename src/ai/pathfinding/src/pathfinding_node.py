#!/usr/bin/python
import rospy

from ai_msgs.srv import Pathfinder, PathfinderResponse, ObjectManager, ObjectManagerResponse,

from robot_watcher.Services import RobotServices

from pathfinding.mesh_generator import MeshMap
from pathfinding.State import ObjectManager
import json

class Pathfinding(object):

    def __init__(self):
        rospy.init_node("path_node", log_level=rospy.INFO)

		rospy.Service("pathfinding/object", ObjectManager, self.object_srv)
		rospy.Service("pathfinding/get", Pathfinder, self.get_path)

        self.obstacles = {}
        # grid_len = rospy.get_param("~grid_len", 10)
        # data_file = rospy.get_param("~map_file")
        data = json.load(open(rospy.get_param("~map_file")))
        # self.map=MeshMap(data["table"], rospy.get_param("~grid_len", 10))
        for obj in data["object"]:
            self.obstacles[obj["name"]] = obj["points"]
            # map.add_solid(obj["points"])

        # RobotServices.service_ready("ai", "pathfiding", True)
        # rospy.spin()

    def add_object(self, points, name):
        self.obstacles[name] = points
        # self.map.add_solid(points)

    def remove_object(self, name, dict_del):
        # self.map.remove
        if dict_del
            del self.obstacles[name]

    def update_object(self, points, name):
        self.remove_object(name, False)
        self.add_object(name)

    def get_path(self, msg):
        # self.map.
        return PathfinderResponse()

    def object_srv(self, msg):
        if msg.action == ObjectManager.ADD:
            self.add_object(msg.points, msg.name)
        elif msg.action == ObjectManager.DELETE:
            self.remove_object(msg.name, True)
        elif msg.action == ObjectManager.UPDATE:
            self.remove_object(msg.name, False)
            self.add_object(msg.points, msg.name)
        else:
            rospy.logwarn("Pathfinding : object action {} unknow".format(msg.action))

        return ObjectManagerResponse()


# get path: pos

# set object: pos name

# remove object : name

# update object : new_pos, name

if __name__ == '__main__':
    Pathfinding()
