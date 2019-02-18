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
        
def getPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def getPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def getPoseBlock(data):
    global BlockPose
    BlockPose = data
    
def hoverOverPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z += 0.25
	return newPose
	
def grabPose(poseStmpd):
	newPose = copy.deepcopy(poseStmpd)
	newPose.pose.position.z -= 0.02
	return newPose
	

def handle_ObtainObject(req):
    
    global limb
    limb = req.limb
    global object_name
    object_name = req.objectName
    poseTo = None
    
    if object_name == "left_button":
        poseTo = LeftButtonPose
    elif object_name == "right_button":
        poseTo = RightButtonPose
    else:
        poseTo = BlockPose


    move_to_start(limb)
    toggle_gripper_state(limb, 'open')
    approach(limb, hoverOverPose(poseTo))
    approach(limb, grabPose(poseTo))
    toggle_gripper_state(limb, 'close')
    approach(limb, hoverOverPose(poseTo)) 
    move_to_start(limb)
    
    return ObtainObjectSrvResponse(1)


def main():
    rospy.init_node("obtain_object_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('move_to_start_srv', timeout=60)
    rospy.wait_for_service('toggle_gripper_state_srv', timeout=60)
    rospy.wait_for_service('approach_srv', timeout=60)

    rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("obtain_object_srv", ObtainObjectSrv, handle_ObtainObject)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
