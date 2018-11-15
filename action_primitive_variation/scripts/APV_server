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


actionToVary = None 

def handle_APV(req):
    print("Received:")
    print("Action to Vary: ")
    print(req.actionName)
    
    
    global actionToVary
    actionToVary = req.actionToVary


    # return PressButtonSrvResponse(1)


def main():
    rospy.init_node("press_button_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("APV_srv", PressButtonSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
