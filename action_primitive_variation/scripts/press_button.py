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
#import action_primitive_variation

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
limb = None
button_name = None
poseStampedTo = None

class PressButton(object):
    def __init__(self, limb, hover_distance = 0.01, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(pose)
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.pose.position.z = approach.pose.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
        
    def approach(self, pose):
        appr = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        appr.pose.position.z = appr.pose.position.z + self._hover_distance
        joint_angles = self.ik_request(appr)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()



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
	

def handle_pressButton(req):
    print("Received:")
    print("PoseStamped:")
    print(req.buttonPoseStamped)
    print("Limb:")
    print(req.limb)
    print("Button name:")
    print(req.buttonName)
    
    
    global limb
    limb = req.limb
    global button_name
    button_name = req.buttonName
    global poseStampedTo
    poseStampedTo = req.buttonPoseStamped
    
    
    hover_distance = 0.15
    
    if limb == 'left':
		starting_joint_angles_l = {'left_w0': 0.6699952259595108,
								   'left_w1': 1.030009435085784,
                                   'left_w2': -0.4999997247485215,
                                   'left_e0': -1.189968899785275,
                                   'left_e1': 1.9400238130755056,
                                   'left_s0': -0.08000397926829805,
                                   'left_s1': -0.9999781166910306}
    else:
        starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                   'right_e1': 1.9341522973651006,
                                   'right_s0': 0.936293285623961,
                                   'right_s1': -0.9939970420424453,
                                   'right_w0': 0.27417171168213983,
                                   'right_w1': 0.8298780975195674,
                                   'right_w2': -0.5085333554167599}
    
    currentAction = PressButton(limb, hover_distance)
    
    if limb == 'left':
        currentAction.move_to_start(starting_joint_angles_l)
    else:
        currentAction.move_to_start(starting_joint_angles_r)
        
    currentAction.gripper_close()
    currentAction.approach(hoverOverPose(LeftButtonPose))
    currentAction.approach(LeftButtonPose)
    currentAction.approach(hoverOverPose(LeftButtonPose))
    if limb == 'left':
        currentAction.move_to_start(starting_joint_angles_l)
    else:
        currentAction.move_to_start(starting_joint_angles_r)
    
    return PressButtonSrvResponse(1)


def main():
    rospy.init_node("press_button_node")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("block3_pose", PoseStamped, getPoseButtonLeft)
    rospy.Subscriber("block2_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block1_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("PressButtonSrv", PressButtonSrv, handle_pressButton)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
