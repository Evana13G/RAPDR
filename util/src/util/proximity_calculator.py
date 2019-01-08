
#!/usr/bin/env python

import sys
import cv2
import math
import time

import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

# from baxter_core_msgs.srv import (
#     SolvePositionIK,
#     SolvePositionIKRequest,
# )

from tf.transformations import *
# import baxter_interface
# from environment.srv import *
# from environment.msg import *
# from util.image_converter import ImageConverter

def is_touching(object1_loc, object2_loc, epsilon=0.135):
    if ((object1_loc is not None) and (object2_loc is not None)):
        obj1 = np.array((object1_loc.pose.position.x, 
                            object1_loc.pose.position.y,
                            object1_loc.pose.position.z)) 
        obj2 = np.array((object2_loc.pose.position.x, 
                            object2_loc.pose.position.y,
                            object2_loc.pose.position.z)) 
        if np.linalg.norm(obj1 - obj2) < epsilon:
            return True 
        return False