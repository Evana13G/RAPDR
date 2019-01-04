

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
from gazebo_msgs.msg import (
    LinkState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    
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


class ImageConverter:
    def __init__(self):
        print("Inside Constructor. Pubs and subs set up")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/head_camera/image", Image, self.callbackImage)

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("Successful Conversion")
        except CvBridgeError as e:
            print(e)
        frame = cv_image
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40,40,40])
        upper_green = np.array([200,255,255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        print(np.count_nonzero(mask))