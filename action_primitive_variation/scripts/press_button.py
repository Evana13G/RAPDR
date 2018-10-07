#!/usr/bin/env python

import argparse
import struct
import sys
import copy

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

import baxter_interface

LeftButtonPose = None
RightButtonPose = None
BlockPose = None


def move_to_start(start_angles=None):
    if not start_angles:
        start_angles = dict(zip(_joint_names, [0]*7))
    guarded_move_to_joint_position(start_angles)
    gripper_open()
    rospy.sleep(1.0)
    print("Running. Ctrl-c to quit")

def ik_request(pose):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    try:
        resp = _iksvc(ikreq)
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
        if verbose:
            print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                     (seed_str)))
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if verbose:
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        return False
    return limb_joints

def guarded_move_to_joint_position(joint_angles):
    if joint_angles:
        limb.move_to_joint_positions(joint_angles)
    else:
        rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

def gripper_open():
    gripper.open()
    rospy.sleep(1.0)

def gripper_close():
    gripper.close()
    rospy.sleep(1.0)

def approach(pose):
    approach = copy.deepcopy(pose)
    # approach with a pose the hover-distance above the requested pose
    approach.position.z = approach.position.z + _hover_distance
    joint_angles = ik_request(approach)
    guarded_move_to_joint_position(joint_angles)

def retract():
    # retrieve current pose from endpoint
    current_pose = _limb.endpoint_pose()
    ik_pose = Pose()
    ik_pose.position.x = current_pose['position'].x
    ik_pose.position.y = current_pose['position'].y
    ik_pose.position.z = current_pose['position'].z + _hover_distance
    ik_pose.orientation.x = current_pose['orientation'].x
    ik_pose.orientation.y = current_pose['orientation'].y
    ik_pose.orientation.z = current_pose['orientation'].z
    ik_pose.orientation.w = current_pose['orientation'].w
    joint_angles = ik_request(ik_pose)
    guarded_move_to_joint_position(joint_angles)

def servo_to_pose(pose):
    joint_angles = ik_request(pose)
    guarded_move_to_joint_position(joint_angles)
    
def approach(pose):
    appr = copy.deepcopy(pose)
    appr.position.z = appr.position.z + _hover_distance
    joint_angles = ik_request(appr)
    guarded_move_to_joint_position(joint_angles)

def pick(pose):
    gripper_open()
    approach(pose)
    servo_to_pose(pose)
    gripper_close()
    retract()

def place(pose):
    approach(pose)
    servo_to_pose(pose)
    gripper_open()
    retract()

def delete_gazebo_models():
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


def main():
    rospy.init_node("press_button")
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.Subscriber("block3_pose", Pose, getPoseButtonLeft)
    rospy.Subscriber("block2_pose", Pose, getPoseButtonRight)
    rospy.Subscriber("block1_pose", Pose, getPoseBlock)

    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}

    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    block_poses = list()
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.129),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))


    limb_name = 'left'
    hover_distance = 0.15
    verbose = True
    limb = baxter_interface.Limb(limb_name)
    gripper = baxter_interface.Gripper(limb_name)
    ns = "ExternalTools/" + limb_name + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    rospy.wait_for_service(ns, 5.0)
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    move_to_start(starting_joint_angles)


    while not rospy.is_shutdown():
        approach(Pose(position=LeftButtonPose.position, orientation=LeftButtonPose.orientation))
        approach(Pose(position=RightButtonPose.position, orientation=RightButtonPose.orientation))
    return 0

if __name__ == '__main__':
    sys.exit(main())


# TODO:
# Make this a service, where the following arguments are passed
#     limb: right or left
#     button: which button to press, either 1 or 2 
