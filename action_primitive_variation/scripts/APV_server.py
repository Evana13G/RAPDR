#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *

import baxter_interface

from action_primitive_variation.srv import *


actionToVary = None 
gripper = None
obj = None
button = None
actionPrimitiveKnowledgeBase = {}


def handle_APV(req):
    print("(Received) Action to Vary: ")
    print(req.actionName)
    params = {}

    global actionToVary
    actionToVary = req.actionName
    if actionToVary is not None: 
        params['actionName'] = actionToVary
    global gripper
    gripper = req.gripper
    if gripper is not None: 
        params['gripper'] = gripper
    global obj
    obj = req.obj
    if obj is not None: 
        params['obj'] = obj
    global button
    button = req.button
    if button is not None: 
        params['button'] = button

    execute_action(actionToVary, params)


    # return PressButtonSrvResponse(1)

def execute_action(actionName, params):
    args = params.values()
    print(args)
    # rospy.wait_for_service('press_button_srv', timeout=60)
    # try:
    #     b = rospy.ServiceProxy('press_button_srv', PressButtonSrv)
    #     resp = b('left', 'block')
    #     print(resp.success_bool)
    # except rospy.ServiceException, e:
    #     print("Service call failed: %s"%e)

def main():
    rospy.init_node("APV_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    # rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    # rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
