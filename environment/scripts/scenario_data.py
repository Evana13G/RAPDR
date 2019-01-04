#!/usr/bin/env python

from __future__ import print_function, division
import roslib
#roslib.load_manifest('my_package')
import sys
import cv2
import math
import time
from cv_bridge import CvBridge, CvBridgeError

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
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    LinkState,
)
from sensor_msgs.msg import (
    Image,
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

from environment.srv import *
from environment.msg import *

LeftButtonPose = None
RightButtonPose = None
BlockPose = None
LeftGripperPose = None
RightGripperPose = None
TablePose = None
WallPose = None

predicates_publisher = None 

PredicatesList = []

def setPoseButtonLeft(data):
    global LeftButtonPose
    LeftButtonPose = data
    updatePredicates()

def setPoseButtonRight(data):
    global RightButtonPose
    RightButtonPose = data
    updatePredicates()

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    updatePredicates()

def setPoseGripperLeft(data):
    global LeftGripperPose
    LeftGripperPose = data
    updatePredicates()

def setPoseGripperRight(data):
    global RightGripperPose
    RightGripperPose = data
    updatePredicates()

def setPoseTable(data):
    global TablePose
    TablePose = data
    updatePredicates()

def setPoseWall(data):
    global WallPose
    WallPose = data
    updatePredicates()

# This will eventually just take the data to be updated 
def updatePredicates():
    generatePredicates()

def generatePredicates():
    # return the list of predicates 

    # Process the at() predicates
    predicates = []
    predicates.append("at(button_left, (" + str(LeftButtonPose.pose.position.x) + ", " +
                                            str(LeftButtonPose.pose.position.y) + ", " +
                                            str(LeftButtonPose.pose.position.z) + "))")
    predicates_publisher.publish(predicates)
    # predicates.append(At(object="button_left", x=LeftButtonPose.pose.position.x, y=LeftButtonPose.pose.position.y, z=LeftButtonPose.pose.position.z))
    # publist the predicates
    # return ScenarioDataSrvResponse(predicates)



#################################################################################################
###################### move this to a different class ###########################################
class image_converter:
    def __init__(self):
        print("Inside Constructor. Pubs and subs set up")
        self.bridge = CvBridge()
        self.initTime = 0
        self.savedFrames = {}
        self.savedFramesStr = ""
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callbackImage)

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("Successful Conversion")
        except CvBridgeError as e:
            print(e)
        frame = cv_image
        #cv2.imshow('frame', frame)
#        frame = cv2.resize(frame,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
#        frameCopy = frame.copy()
#        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40,40,40])
        upper_green = np.array([200,255,255])
#        lower_red = np.array([0,0,255])
#        upper_red = np.array([150,150, 255])
#
#        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        print(np.count_nonzero(mask))
#        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
#        for cnt in contours:
#            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
#            #print(len(approx))
#            if len(approx)==4:
#                #print("square")
#                cv2.drawContours(frameCopy,[cnt],0,(0,0,255),-1)
#        maskRect = cv2.inRange(frameCopy, lower_red, upper_red)
#
#        kernel = np.ones((5,5), np.uint8)
#        erosion = cv2.erode(maskRect, kernel, iterations = 1)
#        erosionArray = np.asarray(erosion)
#        areaErosion = np.count_nonzero(erosionArray)


###################### move this to a different class ###########################################
#################################################################################################

def main():
    rospy.init_node("scenario_data_node")
    rospy.wait_for_message("/robot/sim/started", Empty)
   
    rospy.Subscriber("left_gripper_pose", Point, setPoseGripperLeft)
    rospy.Subscriber("right_gripper_pose", Point, setPoseGripperRight)
    rospy.Subscriber("left_button_pose", PoseStamped, setPoseButtonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, setPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    rospy.Subscriber("cafe_table_pose", PoseStamped, setPoseTable)
    rospy.Subscriber("grey_wall_pose", PoseStamped, setPoseWall)


    # s = rospy.Service("scenario_data_srv", ScenarioDataSrv, generatePredicates)
    global predicates_publisher 
    predicates_publisher = rospy.Publisher('predicate_values', , queue_size = 10)

    # ic = image_converter()
    
    s = rospy.Service("scenario_data_srv", ScenarioDataSrv, generatePredicates)

    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

