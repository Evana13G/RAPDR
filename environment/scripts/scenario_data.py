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

# from geometry_msgs.msg import (
#     PoseStamped,
#     Pose,
#     Point,
#     Quaternion,
# )
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
from util.proximity_calculator import *

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

WallState = "DOWN"

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
    if 'pressed(left_button)' in predicates_list:
        if WallState is not prevWallState:
            toggleWallState()

def toggleWallState():
    global WallState
    if WallState == "UP":
        print("DROP WALL")
        dropWall()
        WallState = "DOWN"
    else:
        print("DROP WALL")
        raiseWall()
        WallState = "UP"

def raiseWall():
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    
    start_time = rospy.Time(0,0)
    duration = rospy.Duration(-1,0)
    
    try:
        wallUp = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        resp_wallUp = wallUp("joint_wall", 10, start_time, duration)
    except rospy.ServiceException, e:
        rospy.logerr("ApplyJointEffort service call failed: {0}".format(e))

def dropWall():
    rospy.wait_for_service('gazebo/clear_joint_forces')
    try:
        wallDown = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        reps_wallDown = wallDown("joint_wall")
    except rospy.ServiceException, e:
        rospy.logerr("JointRequest to clear_joint_forces service call failed: {0}".format(e))

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

    # Just do blcok here 
    if (imageConverter.getBlockPixelCount() > 0):
        new_predicates.append(Predicate(operator="is_visible", object="block", locationInformation=None)) 
    predicates_list = new_predicates

def updatePhysicalStateBasedPredicates():
    global predicates_list
    new_predicates = []
    for pred in predicates_list:
        if not (pred.operator == "pressed"):
            new_predicates.append(pred)

    if is_touching(LeftGripperPose, LeftButtonPose) or is_touching(RightGripperPose, LeftButtonPose):
        new_predicates.append(Predicate(operator="pressed", object="left_button", locationInformation=None)) 
    if is_touching(LeftGripperPose, RightButtonPose) or is_touching(RightGripperPose, RightButtonPose):
        new_predicates.append(Predicate(operator="pressed", object="right_button", locationInformation=None)) 
    
    predicates_list = new_predicates

def getPredicates(data):
    # data can be the form you want it in, for example PDDL and rounded 
    return ScenarioDataSrvResponse(pddlStringFormat())

def pddlStringFormat():
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ', (' + 
                              str(round(pred.locationInformation.pose.position.x, 2)) + ', ' + 
                              str(round(pred.locationInformation.pose.position.y, 2)) + ', ' + 
                              str(round(pred.locationInformation.pose.position.z, 2)) + '))')
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

    rospy.Subscriber("left_gripper_pose", PoseStamped, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, setPoseGripperRight)
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

