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

from environment.srv import *
from environment.msg import *
from util.image_converter import ImageConverter

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

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
    # updatePredicates("at", "left_gripper", data)    # need to change return type to poseStamped, instead of Point

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    # updatePredicates("at", "right_button", data)    # need to change return type to poseStamped, instead of Point

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates("at", "table", data)

def setPoseWall(data):
    global WallPose
    WallPose = data
    updatePredicates("at", "wall", data)


def updatePredicates(oprtr, obj, locInf):
    updateLocationPredicates(oprtr, obj, locInf)
    updateVisionBasedPredicates()
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

    # Just do blcok here 
    if (imageConverter.getBlockPixelCount() > 0):
        new_predicates.append(Predicate(operator="is_visible", object="block", locationInformation=None)) 
    predicates_list = new_predicates

def getPredicates(data):
    # data can be the form you want it in, for example PDDL and rounded 
    return ScenarioDataSrvResponse(pddlStringFormat())

def pddlStringFormat():
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ", (" + 
                              str(round(pred.locationInformation.pose.position.x, 2)) + ", " + 
                              str(round(pred.locationInformation.pose.position.y, 2)) + ", " + 
                              str(round(pred.locationInformation.pose.position.z, 2)) + "))")
        else:
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ')')
    return stringList

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
   
    global predicatesPublisher 
    global imageConverter 

    predicatesPublisher = rospy.Publisher('predicate_values', PredicateList, queue_size = 10)
    imageConverter = ImageConverter()

    rospy.Subscriber("left_gripper_pose", Point, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", Point, setPoseGripperRight)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)
    rospy.Subscriber("grey_wall_pose", PoseStamped, setPoseWall)

    s = rospy.Service("scenario_data_srv", ScenarioDataSrv, getPredicates)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

