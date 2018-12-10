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

from action_primitive_variation.srv import *


LeftButtonPose = None
RightButtonPose = None
BlockPose = None
#Left Gripper Left Finger Pose
lglfPose = None
#Left Gripper Right Finger Pose
lgrfPose = None
#Right Gripper Left Finger Pose
rglfPose = None
#Right Gripper Right Finger Pose
rgrfPose = None
#Left Gripper Pose
leftGripperPose = None
#Right Gripper Pose
rightGripperPose = None

def getPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data

def getPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data

def getPoseBlock(data):
    global BlockPose
    BlockPose = data



def getLeftGripper():
    rospy.wait_for_service('/gazebo/get_link_state')
    
    try:
        lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_lglf_link_state = lglf_link_state('l_gripper_l_finger', 'world')
        global lglfPose
        lglfPose = resp_lglf_link_state.link_state.pose.position
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))
    print("LGLF pose: ")
    print(lglfPose)

    try:
        lgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_lgrf_link_state = lgrf_link_state('l_gripper_r_finger', 'world')
        global lgrfPose
        lgrfPose = resp_lgrf_link_state.link_state.pose.position
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for l_gripper_r_finger: {0}".format(e))
    print("LGRF pose: ")
    print(lgrfPose)

    global leftGripperPose
    leftGripperPose = Point()
    leftGripperPose.x = (lglfPose.x + lgrfPose.x)/2
    leftGripperPose.y = (lglfPose.y + lgrfPose.y)/2
    leftGripperPose.z = (lglfPose.z + lgrfPose.z)/2

    print("Left Gripper Pose: ")
    print(leftGripperPose)

def getRightGripper():
    rospy.wait_for_service('/gazebo/get_link_state')
    
    try:
        rglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_rglf_link_state = rglf_link_state('r_gripper_l_finger', 'world')
        global rglfPose
        rglfPose = resp_rglf_link_state.link_state.pose.position
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_l_finger: {0}".format(e))
    print("RGLF pose: ")
    print(rglfPose)

    try:
        rgrf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp_rgrf_link_state = rgrf_link_state('r_gripper_r_finger', 'world')
        global lgrfPose
        rgrfPose = resp_rgrf_link_state.link_state.pose.position
    except rospy.ServiceException, e:
        rospy.logerr("get_link_state for r_gripper_r_finger: {0}".format(e))
    print("RGRF pose: ")
    print(rgrfPose)

    global rightGripperPose
    rightGripperPose = Point()
    rightGripperPose.x = (rglfPose.x + rgrfPose.x)/2
    rightGripperPose.y = (rglfPose.y + rgrfPose.y)/2
    rightGripperPose.z = (rglfPose.z + rgrfPose.z)/2

    print("Right Gripper Pose: ")
    print(rightGripperPose)



def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
    
    rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)

    # rospy.wait_for_service('/gazebo/get_link_state')
    
    # try:
    #     lglf_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    #     resp_lglf_link_state = lgrf_link_state('l_gripper_l_finger', 'world')
    #     global lglfPose
    #     lglfPose = resp_lglf_link_state.pose

    #     # cafe_table_ms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #     # resp_cafe_table_ms = cafe_table_ms("cafe_table", "");
    #     # pose_cafe_table = resp_cafe_table_ms.pose
    #     # header_cafe_table = resp_cafe_table_ms.header
    #     # header_cafe_table.frame_id = frameid_var
    #     # poseStamped_cafe_table = PoseStamped(header=header_cafe_table, pose=pose_cafe_table)
    #     # pub_cafe_table_pose.publish(poseStamped_cafe_table)
    # except rospy.ServiceException, e:
    #     rospy.logerr("get_link_state for l_gripper_l_finger: {0}".format(e))

    # global lglfPose
    getLeftGripper()
    getRightGripper()
            
    

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())




