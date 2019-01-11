#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import os 
import copy

import rospy
import rospkg
import rosbag

from std_msgs.msg import (
    Header,
    Empty,
)
from geometry_msgs.msg import (
    PoseStamped,
    # Pose,
    # Point,
    # Quaternion,
)
from tf.transformations import *

from util.knowledge_base import KnowledgeBase
from util.data_conversion import is_touching
from action_primitive_variation.srv import *
from agent.srv import * 
from util.physical_agent import PhysicalAgent
from pddl.srv import *

actionToVary = None 
gripper = None
obj = None
button = None

# AP_names = ['press_button', 'obtain_object']
# AP_services = ['press_button_srv', 'obtain_object_srv']
# AP_srvs = [PressButtonSrv, ObtainObjectSrv]

KB = KnowledgeBase()


global LeftButtonPose
global LeftGripperPose

############### START: ROSbag handling
def handle_buttonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def handle_gripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data

def execute_action(req):
    # Ping service to get start end
    # b = rospy.ServiceProxy(KB.getService(actionName), KB.getSrv(actionName))
    # resp = None
    # rospy.wait_for_service(KB.getService(actionName), timeout=60)

    # Assuming I get initial position and final position
    pa = PhysicalAgent(limb="left", hover_distance=0.0) # 



    pose_initial = req.initPosition
    pose_final = req.endPosition


    # pose_initial = LeftGripperPose
    # pose_final = LeftButtonPose
    # print(pose_initial)
    # print(pose_final)

    pa.approach(pose_initial)
    print("Got to init pose")

    if not is_touching(pose_initial, pose_final): # Maybe check distance between poses to check if they're close enough
        print("checked to see if the start and end pos were equal")
        pa.approach(pose_final)

    try:
        return PartialPlanExecutorSrvResponse(1)
    except rospy.ServiceException, e:
        return PartialPlanExecutorSrvResponse(0)
        print("Service call failed: %s"%e)


def main():
    rospy.init_node("partial_plan_executor_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)

    rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)
    rospy.Subscriber("left_gripper_pose", PoseStamped, handle_gripperLeft)

    s = rospy.Service("partial_plan_executor_srv", PartialPlanExecutorSrv, execute_action) #rewrite
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
