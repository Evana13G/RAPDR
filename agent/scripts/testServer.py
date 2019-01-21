#!/usr/bin/env python

from agent.srv import PressButtonSrv, CloseGripperSrv, OpenGripperSrv, ObtainObjectSrv, PartialPlanExecutorSrv
from action_primitive_variation.srv import APVSrv

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


# LeftButtonPose = None
# LeftGripperPose = None

# def handle_buttonLeft(data):
#     global LeftButtonPose
#     LeftButtonPose = data

# def handle_gripperLeft(data):
#     global LeftGripperPose
#     LeftGripperPose = data


def main():
    rospy.init_node("agent_test_node")

    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('APV_srv', timeout=60)

    try:
        b_1 = rospy.ServiceProxy('APV_srv', APVSrv)
        b_2 = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
    
        print("..Trying APV...")
        resp_1 = b_1('press_button', 'left', 'left_button', None)
        print(str(len(resp_1.endEffectorInfo)) + "total change points found")
        print("..Trying Partial Plan Executor...")
        i = 0
        while (i + 1) <= len(resp_1.endEffectorInfo) - 1:
            print("Starting iteration " + str(i+1))
            resp = b_2(resp_1.endEffectorInfo[i], resp_1.endEffectorInfo[i+1])
            print("Success bool for iteration " + str(i+1) + ": " + str(resp.success_bool))
            i = i + 1 

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
