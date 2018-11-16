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

from agent.srv import *
from physical_agent import PhysicalAgent


LeftButtonPose = None
RightButtonPose = None
BlockPose = None
limb = None
button_name = None
poseStampedTo = None

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
	

def handle_closeGripper(req):
    print("Limb:")
    print(req.limb)
    
    global limb
    limb = req.limb    
    hover_distance = 0.15
    currentAction = PhysicalAgent(limb, hover_distance)
    currentAction.gripper_close()
    return CloseGripperSrvResponse(1)


def main():
    rospy.init_node("close_gripper_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    s = rospy.Service("CloseGripperSrv", CloseGripperSrv, handle_closeGripper)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
