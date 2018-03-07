#!/usr/bin/env python
import rospy
import time
from sender.msg import test
from ai_robot_status.msg import NodeReadiness

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s and", data.name)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('receiver', anonymous=True)

    rospy.Subscriber("chatter", test, callback)
    pub = rospy.Publisher("/ai/robot_watcher/node_readiness", NodeReadiness, queue_size = 10)
    msg = NodeReadiness("/receiver", True)
    time.sleep(2)
    pub.publish(msg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
