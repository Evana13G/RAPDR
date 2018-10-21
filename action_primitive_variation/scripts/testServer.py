#!/usr/bin/env python

from action_primitive_variation.srv import PressButtonSrv
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

LeftButtonPose = None
RightButtonPose = None
    
def getPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def getPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data


def main():
    rospy.init_node("test_node")
    rospy.Subscriber("block3_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("block2_pose", PoseStamped, getPoseButtonRight)
    rospy.wait_for_service('PressButtonSrv', timeout=60)
    print("Ready to call service")
    try:
        b = rospy.ServiceProxy('PressButtonSrv', PressButtonSrv)
        resp = b('left', 'LeftButton', LeftButtonPose)
        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    main()
