#!/usr/bin/env python

from scenario_data.srv import *
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
    rospy.wait_for_service('ScenarioSrv', timeout=60)
    try:
        b = rospy.ServiceProxy('ScenarioSrv', ScenarioSrv)
        resp = b('predicate')
        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
