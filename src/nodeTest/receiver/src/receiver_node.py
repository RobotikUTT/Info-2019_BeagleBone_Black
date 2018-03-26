#!/usr/bin/env python
import rospy
import time
from sender.msg import test, test2
from robot_watcher.srv import NodeReadiness
from robot_watcher.Services import RobotServices

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s and", data.name)

def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + " Can message heard; mode : {}   data : {}".format(data.mode, str([ n for n in data.data])))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('receiver')

    rospy.Subscriber("chatter", test, callback)
    rospy.Subscriber("receiver/test", test2, callback2)
    # pub = rospy.Publisher("/ai/robot_watcher/node_readiness", NodeReadiness, queue_size = 10)
    # msg = NodeReadiness("/receiver", True)
    # time.sleep(2)
    # pub.publish(msg)

    rospy.set_param("relative", 99)
    rospy.set_param("/global", 99)
    rospy.set_param("~private", 99)

    RobotServices.service_ready("sender", "", True)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
