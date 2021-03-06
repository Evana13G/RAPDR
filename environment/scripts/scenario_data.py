#!/usr/bin/env python

from __future__ import print_function, division
import roslib
#roslib.load_manifest('my_package')
import sys
import cv2
import math
import time
from cv_bridge import CvBridge, CvBridgeError

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
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    
)
from sensor_msgs.msg import (
    Image,
)
from gazebo_msgs.srv import (
    ApplyJointEffort,
    JointRequest,
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

from environment.srv import *
from environment.msg import *
from util.image_converter import ImageConverter
from util.data_conversion import *
from util.wall_controller import *

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

WallState = "UP"

predicatesPublisher = None 
imageConverter = None 

predicates_list = []

def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data
    updatePredicates("at", "left_button", data)

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data
    updatePredicates("at", "right_button", data)

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    updatePredicates("at", "block", data)

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    updatePredicates("at", "left_gripper", data)    

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    updatePredicates("at", "right_gripper", data)    

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates("at", "table", data)

def setPoseWall(data):
    global WallPose
    WallPose = data
    updatePredicates("at", "wall", data)

def checkElements(prevWallState):
    global WallState
    if 'pressed(left_button)' in pddlStringFormat(predicates_list):
        WallState = "DOWN"
    else:
        WallState = "UP"
    if WallState is not prevWallState:
        toggleWallState()

def toggleWallState():
    if WallState == "DOWN":
        dropWall()
    else:
        raiseWall()

def updatePredicates(oprtr, obj, locInf):
    currWallState = copy.deepcopy(WallState)
    updateLocationPredicates(oprtr, obj, locInf)
    updateVisionBasedPredicates()
    updatePhysicalStateBasedPredicates()
    checkElements(currWallState)
    predicatesPublisher.publish(predicates_list)

def updateLocationPredicates(oprtr, obj, locInf):
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not((pred.operator == oprtr) and (pred.object == obj)):
            new_predicates.append(pred)
    new_predicates.append(Predicate(operator=oprtr, object=obj, locationInformation=locInf)) 
    predicates_list = new_predicates

def updateVisionBasedPredicates():
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not (pred.operator == "is_visible"):
            new_predicates.append(pred)

    # Just do block here 
    if (imageConverter.getBlockPixelCount() > 0):
        new_predicates.append(Predicate(operator="is_visible", object="block", locationInformation=None)) 
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if ((not (pred.operator == "pressed")) and (not (pred.operator == "obtained"))):
            new_predicates.append(pred)
    if is_touching(LeftGripperPose, LeftButtonPose) or is_touching(RightGripperPose, LeftButtonPose):
        new_predicates.append(Predicate(operator="pressed", object="left_button", locationInformation=None)) 
    if is_touching(LeftGripperPose, RightButtonPose) or is_touching(RightGripperPose, RightButtonPose):
        new_predicates.append(Predicate(operator="pressed", object="right_button", locationInformation=None)) 
    if is_obtained(BlockPose, WallPose):
        new_predicates.append(Predicate(operator="obtained", object="block", locationInformation=None))
    
    predicates_list = new_predicates

def getPredicates(data):
    return ScenarioDataSrvResponse(pddlStringFormat(predicates_list), 
                                   pddlObjectsStringFormat(predicates_list),
                                   pddlInitStringFormat(predicates_list),
                                   PredicateList(predicates_list))

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
   
    global predicatesPublisher 
    global imageConverter 

    predicatesPublisher = rospy.Publisher('predicate_values', PredicateList, queue_size = 10)
    imageConverter = ImageConverter()

    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)
    rospy.Subscriber("grey_wall_pose", PoseStamped, setPoseWall)

    rospy.sleep(1)
    s = rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

