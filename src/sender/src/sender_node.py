#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from sender.msg import test
from ai_robot_status.srv import NodeReadiness
from ai_robot_status import RobotServices


def talker():
    pub = rospy.Publisher('chatter', test, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    RobotServices.service_ready("receiver", "", True)
    while not rospy.is_shutdown():
        msg = test()
        msg.value = False
        msg.name = "sender/send"
        # rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass    