#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg
import rosbag

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
)

from std_msgs.msg import (
    Header,
    Empty,
)

class RosBag(object):
    def __init__(self, bagName):
        self.name = bagName
        self.bag = rosbag.Bag(bagName+'.bag','w')

    # def getNumTrajs(self):
    #     if self.name == 'jointState':
    #         return 7
    #     else:
    #         return 0

    def getBag(self):
        return self.bag

    def closeBag(self):
        self.bag.close()

    def openBag(self):
        self.bag = rosbag.Bag(self.name+'.bag','w')

    def writeToBag(self, topic, data):
        self.bag.write(topic, data)

    def getVisualizableData(self):
        # Indexes
        if self.name == 'jointState':

            # hardcoded indices 
            i_left_e0 = 3
            i_left_e1 = 4
            i_left_s0 = 5
            i_left_s1 = 6
            i_left_w0 = 7
            i_left_w1 = 8
            i_left_w2 = 9
            i_right_e0 = 12
            i_right_e1 = 13
            i_right_s0 = 14
            i_right_s1 = 15
            i_right_w0 = 16
            i_right_w1 = 17
            i_right_w2 = 18

            # Arrays of vals to return 
            left_e0 = []
            left_e1 = []
            left_s0 = []
            left_s1 = []
            left_w0 = []
            left_w1 = []
            left_w2 = []
            right_e0 = []
            right_e1 = []
            right_s0 = []
            right_s1 = []
            right_w0 = []
            right_w1 = []
            right_w2 = []


            for topic, msg, t in self.bag.read_messages(topics=['robot/joint_states']):
                left_e0.append(msg.position[i_left_e0])
                left_e1.append(msg.position[i_left_e1])
                left_s0.append(msg.position[i_left_s0])
                left_s1.append(msg.position[i_left_s1])
                left_w0.append(msg.position[i_left_w0])
                left_w1.append(msg.position[i_left_w1])
                left_w2.append(msg.position[i_left_w2])
                right_e0.append(msg.position[i_right_e0])
                right_e1.append(msg.position[i_right_e1])
                right_s0.append(msg.position[i_right_s0])
                right_s1.append(msg.position[i_right_s1])
                right_w0.append(msg.position[i_right_w0])
                right_w1.append(msg.position[i_right_w1])
                right_w2.append(msg.position[i_right_w2])
            
            halfLen = len(left_e0)/2

            left_e0 = left_e0[:halfLen]
            left_e1 = left_e1[:halfLen]
            left_s0 = left_s0[:halfLen]
            left_s1 = left_s1[:halfLen]
            left_w0 = left_w0[:halfLen]
            left_w1 = left_w1[:halfLen]
            left_w2 = left_w2[:halfLen]
            right_e0 = right_e0[:halfLen]
            right_e1 = right_e1[:halfLen]
            right_s0 = right_s0[:halfLen]
            right_s1 = right_s1[:halfLen]
            right_w0 = right_w0[:halfLen]
            right_w1 = right_w1[:halfLen]
            right_w2 = right_w2[:halfLen]

            return [left_e0, left_e1, left_s0, left_s1, left_w0, left_w1, left_w2]
                    # right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2]

        elif self.name == 'predicate':
            
            obj_loc_x = []
            obj_loc_y = []
            obj_loc_z = []
            l_btn_pressed = []
            r_btn_pressed = []
            obj_vis = []

            for topic, msg, t in self.bag.read_messages(topics=['predicate_values']):

                for pred in msg.predicates:
                    if pred.operator == "at":
                        if pred.object == "block":
                            obj_loc_x.append(pred.locationInformation.pose.position.x)
                            obj_loc_y.append(pred.locationInformation.pose.position.y)
                            obj_loc_z.append(pred.locationInformation.pose.position.z)
                    if pred.operator == "pressed":
                        if pred.object == "left_button":
                            l_btn_pressed.append('1')
                        else: 
                            l_btn_pressed.append('0')
                        if pred.object == "right_button":
                            r_btn_pressed.append('1')
                        else: 
                            r_btn_pressed.append('0')
                    if pred.operator == "is_visible":
                        if pred.object == "block":
                            obj_vis.append('1')
                        else: 
                            obj_vis.append('0')

            
            # halfLen = len(left_e0)/2

            # left_e0 = left_e0[:halfLen]
            # left_e1 = left_e1[:halfLen]
            # left_s0 = left_s0[:halfLen]
            # left_s1 = left_s1[:halfLen]
            # left_w0 = left_w0[:halfLen]
            # left_w1 = left_w1[:halfLen]
            # left_w2 = left_w2[:halfLen]
            # right_e0 = right_e0[:halfLen]
            # right_e1 = right_e1[:halfLen]
            # right_s0 = right_s0[:halfLen]
            # right_s1 = right_s1[:halfLen]
            # right_w0 = right_w0[:halfLen]
            # right_w1 = right_w1[:halfLen]
            # right_w2 = right_w2[:halfLen]

            return [obj_loc_x, obj_loc_y, obj_loc_z, l_btn_pressed, r_btn_pressed, obj_vis]
        else: 
            return []