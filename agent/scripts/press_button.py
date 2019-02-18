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

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
limb = None
button_name = None
poseStampedTo = None

move_to_start = rospy.ServiceProxy("move_to_start_srv", MoveToStartSrv)
toggle_gripper_state = rospy.ServiceProxy("toggle_gripper_state_srv", ToggleGripperStateSrv)
approach = rospy.ServiceProxy("approach_srv", ApproachSrv)
    


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
        
def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    
def hoverOverPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z += 0.25
	return newPose
	

def handle_pressButton(req):
    
    global limb
    limb = req.limb
    global button_name
    button_name = req.buttonName
    poseTo = None
    
    if button_name == "left_button":
        poseTo = LeftButtonPose
    elif button_name == "right_button":
        poseTo = RightButtonPose
    else:
        poseTo = BlockPose
            
    move_to_start(limb)
    toggle_gripper_state(limb, 'close')
    approach(limb, hoverOverPose(poseTo))
    approach(limb, poseTo)
    approach(limb, hoverOverPose(poseTo)) 
    move_to_start(limb)

    return PressButtonSrvResponse(1)


def main():
    rospy.init_node("press_button_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)

    rospy.wait_for_service('move_to_start_srv', timeout=60)
    rospy.wait_for_service('toggle_gripper_state_srv', timeout=60)
    rospy.wait_for_service('approach_srv', timeout=60)
    
    s = rospy.Service("press_button_srv", PressButtonSrv, handle_pressButton)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
