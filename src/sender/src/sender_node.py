#!/usr/bin/env python
# license removed for brevity
import rospy
from sender.msg import test, test2
from ai_robot_status.srv import NodeReadiness
from ai_robot_status.msg import RobotStatus
from ai_robot_status.Services import RobotServices
from ai_robot_status.RStatus.State import RobotState

class TalkerNode(object):
    """docstring for TalkerNode"""
    def __init__(self):
        self.robot_status = RobotState.ROBOT_INIT
        pub = rospy.Publisher('chatter', test, queue_size=10)
        pub2 = rospy.Publisher('/ros_can/interface/test', test2, queue_size=10)
        rospy.Subscriber("/ai/robot_watcher/robot_status", RobotStatus, self.get_robot_status)
        rospy.init_node('sender', anonymous=True)
        rate = rospy.Rate(1) # 1hz

        RobotServices.service_ready("receiver", "", True)
        while not rospy.is_shutdown():
            if self.robot_status != RobotState.ROBOT_HALT:
                msg = test()
                msg.value = False
                msg.name = "sender/send"
                # rospy.loginfo(msg)
                pub.publish(msg)

                pub2.publish(test2(25, [1,2,3,4,5,6,7]))
            rate.sleep()
        
    

    def get_robot_status(self, msg):
        # print("callback")
        self.robot_status = msg.robot_status

if __name__ == '__main__':
    try:
        TalkerNode()
    except rospy.ROSInterruptException:
        pass    