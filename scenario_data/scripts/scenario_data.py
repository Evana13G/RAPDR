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


def getPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def getPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def getPoseBlock(data):
    global BlockPose
    BlockPose = data







def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())




