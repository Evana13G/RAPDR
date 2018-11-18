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

from util.knowledge_base import KnowledgeBase

from action_primitive_variation.srv import *

actionToVary = None 
gripper = None
obj = None
button = None

AP_names = ['press_button', 'obtain_object']
AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]
KB = KnowledgeBase(AP_names, AP_services, AP_srvs)


def handle_APV(req):

    params = []

    global actionToVary
    actionToVary = req.actionName
    global gripper
    gripper = req.gripper
    # print(gripper)
    if gripper is not '': 
        params.append(gripper)
    global obj
    obj = req.obj
    # print(obj)
    if obj is not '': 
        params.append(obj)
    global button
    button = req.button
    # print(button)
    if button is not '': 
        params.append(button)

    # print(params)
    execute_action(actionToVary, params)


    return APVSrvResponse(1)

def execute_action(actionName, params):
    rospy.wait_for_service(KB.getService(actionName), timeout=60)

    # while(1):
        
    try:
        b = rospy.ServiceProxy(KB.getService(actionName), KB.getSrv(actionName))
        resp = None
        if len(params) == 1:
            resp = b(params[0])
        elif len(params) == 2:
            resp = b(params[0], params[1])
        elif len(params) == 3:
            resp = b(params[0], params[1], params[2])
        elif len(params) == 4:
            resp = b(params[0], params[1], params[2], params[3])
        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node("APV_node")
    # rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    # rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    # rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
