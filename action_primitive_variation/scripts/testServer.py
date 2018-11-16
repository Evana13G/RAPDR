#!/usr/bin/env python

from action_primitive_variation.srv import PressButtonSrv, CloseGripperSrv, OpenGripperSrv, ObtainObjectSrv, APVSrv
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

def main():
    rospy.init_node("test_node")
    rospy.wait_for_service('APV_srv', timeout=60)
    try:
        b = rospy.ServiceProxy('APV_srv', APVSrv)
        resp = b('obtain_object', 'left', 'block', None)
        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
