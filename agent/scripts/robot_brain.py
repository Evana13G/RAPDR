#!/usr/bin/env python
# Modified from RethinkRobotics website

import argparse
import struct
import sys
import copy

import pyddl

import rospy
import rospkg
import gazebo_msgs

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    String,
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface


from gazebo_msgs.msg import LinkStates

#################################################
################### STATE INFO ##################
class ScenarioElement(object):
    def __init__(self, _name):
        self.name = _name # String
        self.location = Point()
        self.pressed = False

    def setLocation(self, _location):
        self.location = _location

    def setPressed(self, state_value):
        self.pressed = state_value

    def print_data(self):
        print("Name: ")
        print(self.name)
        print("Location: ")
        print(self.location)
        print("Pressed: ")
        print(self.pressed)

    def getLocation(self):
        return self.location

    def getPressed(self):
        return self.pressed 

#################################################
#################################################

class RobotBrain(object):
    def __init__(self):
        self.robot_name = "Baxter" # string
        # self.objects = ['ground_plane::link', 
        #                 'baxter::base', 
        #                 'baxter::head', 
        #                 'baxter::left_upper_shoulder', 
        #                 'baxter::left_lower_shoulder', 
        #                 'baxter::left_upper_elbow', 
        #                 'baxter::left_lower_elbow', 
        #                 'baxter::left_upper_forearm', 
        #                 'baxter::left_lower_forearm', 
        #                 'baxter::left_wrist', 
        #                 'baxter::l_gripper_l_finger', 
        #                 'baxter::l_gripper_r_finger', 
        #                 'baxter::right_upper_shoulder', 
        #                 'baxter::right_lower_shoulder', 
        #                 'baxter::right_upper_elbow', 
        #                 'baxter::right_lower_elbow', 
        #                 'baxter::right_upper_forearm', 
        #                 'baxter::right_lower_forearm', 
        #                 'baxter::right_wrist', 
        #                 'baxter::r_gripper_l_finger', 
        #                 'baxter::r_gripper_r_finger', 
        #                 'cafe_table::link', 
        #                 'grey_wall::link', 
        #                 'block1::block', 
        #                 'block2::block', 
        #                 'block3::block']

        # self.left_button_location = 0
        # self.right_button_location = 0
        # self.object_location = 0
        # self.grey_wall_location = 0
        # self.cafe_table_location = 0
        # self.r_gripper_location = 0
        # self.l_gripper_location  = 0
        # self.left_button_pressed = False
        # self.right_button_pressed = False

        self.left_button = ScenarioElement("left_button")
        self.right_button = ScenarioElement("right_button")
        self.object = ScenarioElement("object")
        self.grey_wall = ScenarioElement("grey_wall")
        self.cafe_table = ScenarioElement("cafe_table")
        self.r_gripper = ScenarioElement("r_gripper")
        self.l_gripper  = ScenarioElement("l_gripper")

        self.objects = []
        self.objects.append(self.left_button)
        self.objects.append(self.right_button)
        self.objects.append(self.object)
        self.objects.append(self.grey_wall)
        self.objects.append(self.cafe_table)
        self.objects.append(self.r_gripper)
        self.objects.append(self.l_gripper)


    def observe_state(self):
        self.evaluate_discrete_data()

    def print_data(self):

        for obj in self.objects:
            obj.print_data()

        # if(self.left_button_pressed):
        #     print("Left Button Pressed")

        # if(self.right_button_pressed):
        #     print("Right Button Pressed")

        # print("Left Button Location: ")
        # print(self.left_button_location)

        # print("Right Button Location: ")
        # print(self.right_button_location)

        # print("Object Location: ")
        # print(self.object_location)

        # print("Grey Wall Location: ")
        # print(self.grey_wall_location)
        
        # print("Cafe Table Location: ")
        # print(self.cafe_table_location)

        # print("Right Gripper Location: ")
        # print(self.r_gripper_location)

        # print("Left Gripper Location: ")
        # print(self.l_gripper_location)

    def evaluate_discrete_data(self):
        if(self.is_button_pressed(self.l_gripper, self.left_button)):
            self.left_button.setPressed(True)
            print("Left Button Pressed: ")
            print(self.left_button.getPressed())
        else:
            self.left_button.setPressed(False)
        if(self.is_button_pressed(self.r_gripper, self.left_button)):
            self.left_button.setPressed(True)
            print("Left Button Pressed: ")
            print(self.left_button.getPressed())
        else:
            self.left_button.setPressed(False)
        if(self.is_button_pressed(self.l_gripper, self.right_button)):
            self.right_button.setPressed(True)
            print("Right Button Pressed: ")
            print(self.right_button.getPressed())
        else:
            self.right_button.setPressed(False)
        if(self.is_button_pressed(self.r_gripper, self.right_button)):
            self.right_button.setPressed(True)
            print("Right Button Pressed: ")
            print(self.right_button.getPressed())
        else:
            self.right_button.setPressed(False)
    # def update(self, data):

    def is_button_pressed(self, gripper, button):
        if((abs(gripper.getLocation().x - button.getLocation().x) < 0.01)
            and (abs(gripper.getLocation().y - button.getLocation().y) < 0.01)
            and (abs(gripper.getLocation().z - button.getLocation().z) < 0.01)):
            return True
        return False

    def set_locations(self, data):
        names = data.name
        poses = data.pose

        try:
            left_button_index = data.name.index("block3::block")
            self.left_button.setLocation(poses[left_button_index].position)
        except:
            left_button_index = -1
            # print("value error for left button index")
            
        try:
            right_button_index = data.name.index("block2::block")
            self.right_button.setLocation(poses[right_button_index].position)
        except:
            right_button_index = -1
            # print("value error for right button index")

        try:
            object_index = data.name.index("block1::block")
            self.object.setLocation(poses[object_index].position)
        except:
            object_index = -1
            # print("value error for object index")

        try:
            grey_wall_index = data.name.index("grey_wall::link")
            self.grey_wall.setLocation(poses[grey_wall_index].position)
        except:
            grey_wall_index = -1
            # print("value error for grey wall index")

        try:
            cafe_table_index = data.name.index("cafe_table::link")
            self.cafe_table.setLocation(poses[cafe_table_index].position)
        except:
            cafe_table_index = -1
            # print("value error for cafe table index")

        try:
            r_gripper_index = data.name.index("baxter::r_gripper_r_finger")
            self.r_gripper.setLocation(poses[r_gripper_index].position)
        except:
            r_gripper_index = -1
            # print("value error for left gripper index")

        try:
            l_gripper_index = data.name.index("baxter::l_gripper_l_finger")
            self.l_gripper.setLocation(poses[l_gripper_index].position)
        except:
            l_gripper_index = -1
            # print("value error for right gripper index")

    def callback(self, data):
        self.set_locations(data)
        self.observe_state()
        # self.print_data()

    def brain(self):
        rospy.init_node("robot_brain", anonymous=True)
        sub = rospy.Subscriber("gazebo/link_states", LinkStates, self.callback)
        rospy.spin() 

if __name__ == '__main__':
    brain = RobotBrain()
    brain.brain()