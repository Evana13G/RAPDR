#!/usr/bin/env python
# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy

from pyddl import Action
from pyddl import neg

import rospy
import rospkg

# import robot_brain

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

class ObtainObject(object):
    def __init__(self, hover_distance = 0.15, verbose=True):
        self._left_limb_name = "left" # string
        self._right_limb_name = "right" # string

        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool

        self._left_limb = baxter_interface.Limb("left")
        self._left_gripper = baxter_interface.Gripper("left")

        self._right_limb = baxter_interface.Limb("right")
        self._right_gripper = baxter_interface.Gripper("right")

        ####################################################################################################
        ns_left = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"  # Not entirely sure what this is 
        self._iksvc_left = rospy.ServiceProxy(ns_left, SolvePositionIK)               # Not entirely sure what this is 
        ns_right = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"  # Not entirely sure what this is 
        self._iksvc_right = rospy.ServiceProxy(ns_right, SolvePositionIK)               # Not entirely sure what this is 
        ####################################################################################################

        rospy.wait_for_service(ns_left, 5.0)
        rospy.wait_for_service(ns_right, 5.0)

        # verify robot is enabled  #########################################################################
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, limb_name, start_angles=None):

        print("Moving the {0} arm to start pose...".format(limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        if(limb_name == "left"):
            self._guarded_move_to_joint_position(self._left_limb, start_angles)
            self.gripper_open(self._left_gripper)
        else:
            self._guarded_move_to_joint_position(self._right_limb, start_angles)
            self.gripper_open(self._right_gripper)
        rospy.sleep(1.0)    
        print("Running. Ctrl-c to quit")


    # Not entirely sure what this is ########################################################################
    # I think its a request for the robot to move to a certain position
    # Takes in a pose and outputs a valid joint solution
    #
    def ik_request(self, gripper, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()

        print("IK REQUEST ***************************")
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            print("GRIPPER NAME: " + gripper.name)
            if(gripper.name == "left_gripper" or gripper.name == "left"):
                # print("IK request is going to be for LEFT GRIPPER")
                resp = self._iksvc_left(ikreq)
            elif (gripper.name == "right_gripper" or gripper.name == "right"):
                # print("IK request is going to be for RIGHT GRIPPER")
                resp = self._iksvc_right(ikreq)  
            else:
                print("Unable to resolve gripper for IK_REQUEST")
                exit(1) 
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # print("IKREQ RESPONSE: ")
        # print(resp)


        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
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
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, limb, joint_angles):
        print("Limb: " + limb.name)
        print("Joint Angles: ")
        print(joint_angles)
        if joint_angles:
            limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self, gripper):
        gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self, gripper):
        gripper.close()
        rospy.sleep(1.0)

    def _approach(self, gripper, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        if (gripper.name == "left_gripper"):
            limb = self._left_limb
        elif (gripper.name == "right_gripper"):
            limb = self._right_limb 
        else:
            print("Unable to resolve gripper for APPROACH")
            exit(1)

        joint_angles = self.ik_request(limb, approach)
        self._guarded_move_to_joint_position(limb, joint_angles)

    def _retract(self, gripper):
        # retrieve current pose from endpoint
        if (gripper.name == "left_gripper"):
            limb = self._left_limb
        elif (gripper.name == "right_gripper"):
            limb = self._right_limb
        else:
            print("Unable to resolve gripper for RETRACT")
            exit(1)

        current_pose = limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        joint_angles = self.ik_request(limb, ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(limb, joint_angles)

    def _servo_to_pose(self, gripper, pose):
        # servo down to release
        joint_angles = self.ik_request(gripper, pose)
        if (gripper == self._left_gripper):
            limb = self._left_limb
        else:
            limb = self._right_limb
        self._guarded_move_to_joint_position(limb, joint_angles)

    def pick(self, gripper, pose):
        # open the gripper
        self.gripper_open(gripper)
        # servo above pose
        self._approach(gripper, pose)
        # servo to pose
        self._servo_to_pose(gripper, pose)
        # close gripper
        self.gripper_close(gripper)
        # retract to clear object
        self._retract(gripper)

    #########################################################################################################
    # Our Action Primitives #################################################################################
    # For PDDL planning
    def push_button(self, button_name, gripper_name, pose):
        if (gripper_name == "right"):
            self._approach(self._right_gripper, pose)
            self._servo_to_pose(self._right_gripper, pose)
        elif (gripper_name == "left"):
            self._approach(self._left_gripper, pose)
            self._servo_to_pose(self._left_gripper, pose)
        else:
            print("Unable to resolve gripper for PUSH_BUTTON")
            exit(1)

    def grab_object(self, object_name, gripper_name, pose):
        print("GRIPPER NAME: " + gripper_name)
        if (gripper_name == "right"):
            self.pick(self._right_gripper, pose)
        elif (gripper_name == "left"):
            self.pick(self._left_gripper, pose)
        else:
            print("Unable to resolve gripper for GRAB_OBJECT")
            exit(1)

    # def move_object(self, object_name, pose):
    #     print("Moving object")

    # def observe_scenario_state():
    #     print("Observing scenario state")
    #     print("Object")   # OBJECT
    #     print("Button 1") # OBJECT
    #     print("Button 2") # OBJECT
    #     print("Wall")     # OBJECT

def main():

    rospy.init_node("obtain_object")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)


    # Use PDDL planner to plan for a goal 
    #
    #

    # robotBrain = RobotBrain()

    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles_left = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}

    starting_joint_angles_right = {'right_w0': -0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': 0.4999997247485215,
                             'right_e0': 1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                             'right_s1': -0.9999781166910306}

    
    oo = ObtainObject(hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)


    oo.move_to_start("left", starting_joint_angles_left)
    oo.move_to_start("right", starting_joint_angles_right)

    idx = 0

    button1_pose = Pose(
        position=Point(x=0.6, y=-0.3, z=-0.09),
        orientation=overhead_orientation)
    button2_pose = Pose(
        position=Point(x=0.6, y=0.13, z=-0.09),
        orientation=overhead_orientation)
    object_c_pose = Pose(
        position=Point(x=0.8, y=-0.01, z=-0.129),
        orientation=overhead_orientation)

    print("\n************************************Obtaining object...")
    try:
        oo.grab_object("object", "left", object_c_pose)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Move to object failed: %s" % (e,))
        print("\nObtaining object FAILED")

    print("\n************************************Pushing button...")
    try: 
        oo.push_button("button1", "right", button1_pose)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Push button failed: %s" % (e,))  
        print("\nPush button FAILED")

    print("\n************************************Removing obstruction...")
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("grey_wall")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

    print("\n************************************Obtaining object...")
    try:
        oo.grab_object("object", "left", object_c_pose)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Move to object failed: %s" % (e,))  
        print("\nObtaining object FAILED")

    return 0

#######################################################################################
################################## Actions and stuff ##################################
#######################################################################################



grabObject = Action(
    'grab_object',
    parameters=(
        ('object_name', 'obj_c'),
        ('gripper_name', 'gripper'),
        ('object_position', 'o_x'),
        ('object_position', 'o_y'),
        ('object_position', 'o_z'),
        ('gripper_position', 'g_x'),
        ('gripper_position', 'g_y'),
        ('gripper_position', 'g_z'),
    ),
    preconditions=(
        ('at', 'obj_c', 'o_x', 'o_y', 'o_z'),
        ('at', 'gripper', 'g_x', 'g_y', 'g_z'),
    ),
    effects=(
        neg(('at', 'gripper', 'g_x', 'g_y', 'g_z')),
        ('at', 'gripper', 'o_x', 'o_y', 'o_z'),
    ),
)

pushButton = Action(
    'push_button',
    parameters=(
        ('button_name', 'button_1'),
        ('gripper_name', 'gripper'),
        ('button_position', 'b_x'),
        ('button_position', 'b_y'),
        ('button_position', 'b_z'),
        ('gripper_position', 'g_x'),
        ('gripper_position', 'g_y'),
        ('gripper_position', 'g_z'),
    ),
    preconditions=(
        ('at', 'button_1', 'b_x', 'b_y', 'b_z'),
        ('at', 'gripper', 'g_x', 'g_y', 'g_z'),
    ),
    effects=(
        neg(('at', 'gripper', 'g_x', 'g_y', 'g_z')),
        ('at', 'gripper', 'o_x', 'o_y', 'o_z'),
    ),
)

# Need to figure out what the predicates are. Right now it looks like 
# there is 

if __name__ == '__main__':
    sys.exit(main())
