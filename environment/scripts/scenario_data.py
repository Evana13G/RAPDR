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
from util.image_converter import ImageConverter

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def setPoseBlock(data):
    global BlockPose
    BlockPose = data

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data

def setPoseTable(data):
    global TablePose
    TablePose = data

def setPoseWall(data):
    global WallPose
    WallPose = data

def generatePredicates(data):
    # return the list of predicates 

    # Process the at() predicates
    predicates = []
    predicates.add("at(button_left, " + str(LeftButtonPose) +")")

    return ScenarioDataSrvResponse(predicates)

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
   
    rospy.Subscriber("left_gripper_pose", Point, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", Point, setPoseGripperRight)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)
    rospy.Subscriber("grey_wall_pose", PoseStamped, setPoseWall)

    ic = ImageConverter()
    s = rospy.Service("scenario_data_srv", ScenarioDataSrv, generatePredicates)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

