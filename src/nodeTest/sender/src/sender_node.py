#!/usr/bin/env python
# license removed for brevity
import rospy
from sender.msg import test, test2
from ai_msgs.srv import NodeReadiness
from ai_msgs.msg import RobotStatus
from robot_watcher.Services import RobotServices
from robot_watcher.RStatus.State import RobotState

class TalkerNode(object):
    """docstring for TalkerNode"""
    def __init__(self):
        self.robot_watcher = RobotState.ROBOT_INIT
        pub = rospy.Publisher('chatter', test, queue_size=10)
        pub2 = rospy.Publisher('/ros_can/interface/test', test2, queue_size=10)
        rospy.Subscriber("/ai/robot_watcher/robot_status", RobotStatus, self.get_robot_watcher)
        rospy.init_node('sender', anonymous=True)
        rate = rospy.Rate(1) # 1hz

        RobotServices.service_ready("receiver", "", True)
        while not rospy.is_shutdown():
            if self.robot_watcher != RobotState.ROBOT_HALT:
                msg = test()
                msg.value = False
                msg.name = "sender/send"
                # rospy.loginfo(msg)
                # pub.publish(msg)

                # pub2.publish(test2(25, [1,2,3,4,5,6,7]))
            rate.sleep()



    def get_robot_watcher(self, msg):
        # print("callback")
        self.robot_watcher = msg.robot_watcher

if __name__ == '__main__':
    try:
        TalkerNode()
    except rospy.ROSInterruptException:
        pass
