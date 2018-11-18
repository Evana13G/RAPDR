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


LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
RobotCurrentView = None


def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data
    return "Left Button position at: " + LeftButtonPose

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data
    return "Right Button position at: " + RightButtonPose

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    return "Block position at: " + BlockPose

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    return "Left Gripper position at: " + LeftGripperPose
    
def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    return "Right Gripper position at: " + RightGripperPose
    
def setRobotCurrView(data):
    global RobotCurrentView
    RobotCurrentView = data
    return "Robot Current View at: " + RobotCurrentView


def getPoseButtonLeft():
    global LeftButtonPose
    return deepcopy(LeftButtonPose)

def getPoseButtonRight():
    global RightButtonPose
    return deepcopy(RightButtonPose)

def getPoseBlock():
    global BlockPose
    return deepcopy(BlockPose)

def getPoseGripperLeft():
    global LeftGripperPose
    return deepcopy(LeftGripperPose)
    
def getPoseGripperRight():
    global RightGripperPose
    return deepcopy(RightGripperPose)
    
def getRobotCurrView():
    global RobotCurrentView
    return deepcopy(RobotCurrentView)


def handle_Scenario(req):
    if (req.returnDataType == "predicate"):
        print(getPoseButtonLeft())
        print(getPoseButtonRight())
        print(getPoseBlock())
        print(getPoseGripperLeft())
        print(getPoseGripperRight())
        print(getRobotCurrView())

    else:
        print("Raw Data")

    return ScenarioSrvResponse(1)


def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    rospy.Subscriber("left_button_pose", PoseStamped, gstPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("robot_current_view", PoseStamped, setRobotCurrView)
    
    s = rospy.Service("ScenarioSrv", ScenarioSrv, handle_Scenario)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())




