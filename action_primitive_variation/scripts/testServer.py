#!/usr/bin/env python

from action_primitive_variation.srv import PressButtonSrv, CloseGripperSrv, OpenGripperSrv, GraspObjectSrv
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


ObjectPose = None
    
def getPoseBlock(data):
    global ObjectPose
    ObjectPose = data


def main():
    rospy.init_node("test_node")
    rospy.Subscriber("block1_pose", PoseStamped, getPoseBlock)
    print("Inside testServer.py")
    rospy.wait_for_service('GraspObjectSrv', timeout=60)
    print("Ready to call service")
    try:
        b = rospy.ServiceProxy('GraspObjectSrv', GraspObjectSrv)
        resp = b('left', 'object', ObjectPose)
        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
