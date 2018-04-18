#!/usr/bin/env python
# license removed for brevity
import rospy
from sender.msg import test, test2
from ai_msgs.srv import NodeReadiness, GetActionToDo
from ai_msgs.msg import RobotStatus
from robot_watcher.Services import RobotServices
from robot_watcher.RStatus.State import RobotState

SRV = "/ai/scheduler/actionToDo"

class TalkerNode(object):
    """docstring for TalkerNode"""
    def __init__(self):
        self.robot_watcher = RobotState.ROBOT_INIT
        pub = rospy.Publisher('chatter', test, queue_size=10)
        pub2 = rospy.Publisher('/ros_can/interface/test', test2, queue_size=10)
        rospy.Subscriber("/ai/robot_watcher/robot_status", RobotStatus, self.get_robot_watcher)
        rospy.init_node('sender', anonymous=True)
        rate = rospy.Rate(0.5) # 1hz

        RobotServices.service_ready("receiver", "", True)
        robot_x = 0
        robot_y = 0
        while not rospy.is_shutdown():
            if self.robot_watcher != RobotState.ROBOT_HALT:
                pass
                # msg = test()
                # msg.value = False
                # msg.name = "sender/send"
                # rospy.loginfo(msg)
                # pub.publish(msg)

                # pub2.publish(test2(25, [1,2,3,4,5,6,7]))

                # rospy.wait_for_service(SRV, timeout = 1 )
                # try:
                #     _ready_srv = rospy.ServiceProxy(SRV, GetActionToDo)
                #     resp = _ready_srv(robot_x, robot_y)
                #     rospy.loginfo("action to do : {} x: {}  y: {}".format(resp.action_name, robot_x, robot_y))
            	# except Exception as e:
                #     rospy.logerr("{} not responding".format(SRV))
                #
                # robot_x = robot_x + 1000
                # if robot_x >=3000:
                #     robot_x = 0
                #     robot_y = robot_y + 500




            rate.sleep()



    def get_robot_watcher(self, msg):
        # print("callback")
        self.robot_watcher = msg.robot_watcher

if __name__ == '__main__':
    try:
        TalkerNode()
    except rospy.ROSInterruptException:
        pass
