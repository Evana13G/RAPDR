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

KB = KnowledgeBase()

approach = rospy.ServiceProxy("approach_srv", ApproachSrv)

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

    pose_initial = req.initPosition
    pose_final = req.endPosition

    approach(req.limb, pose_initial)

    if not is_touching(pose_initial, pose_final): # Maybe check distance between poses to check if they're close enough
        approach(req.limb, pose_final)

    try:
        return PartialPlanExecutorSrvResponse(1)
    except rospy.ServiceException, e:
        return PartialPlanExecutorSrvResponse(0)
        print("Service call failed: %s"%e)


def main():
    rospy.init_node("partial_plan_executor_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.wait_for_service('approach_srv', timeout=60)
    rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)
    rospy.Subscriber("left_gripper_pose", PoseStamped, handle_gripperLeft)

    s = rospy.Service("partial_plan_executor_srv", PartialPlanExecutorSrv, execute_action) #rewrite
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
