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
    GetLinkState,
)
from gazebo_msgs.msg import (
    LinkState,
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


LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

predicates_publisher = None 

PredicatesList = []

def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data
    updatePredicates()

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data
    updatePredicates()

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    updatePredicates()

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    updatePredicates()

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    updatePredicates()

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates()

def setPoseWall(data):
    global WallPose
    WallPose = data
    updatePredicates()

# This will eventually just take the data to be updated 
def updatePredicates():
    generatePredicates()

def generatePredicates():
    # return the list of predicates 

    # Process the at() predicates
    predicates = []
    predicates.append("at(button_left, (" + str(LeftButtonPose.pose.position.x) + ", " +
                                            str(LeftButtonPose.pose.position.y) + ", " +
                                            str(LeftButtonPose.pose.position.z) + "))")
    predicates_publisher.publish(predicates)
    # predicates.append(At(object="button_left", x=LeftButtonPose.pose.position.x, y=LeftButtonPose.pose.position.y, z=LeftButtonPose.pose.position.z))
    # publist the predicates
    # return ScenarioDataSrvResponse(predicates)


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

    # s = rospy.Service("scenario_data_srv", ScenarioDataSrv, generatePredicates)
    global predicates_publisher 
    predicates_publisher = rospy.Publisher('predicate_values', , queue_size = 10)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())




