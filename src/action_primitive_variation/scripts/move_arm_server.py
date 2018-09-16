#!/usr/bin/env python

from action_primitive_variation.srv import *
import rospy

def handle_move_arm(req):
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return MoveArmResponse(True)

def move_arm_server():
    rospy.init_node('move_arm_server')
    s = rospy.Service('move_arm', MoveArm, handle_move_arm)
    print "Ready to move arm."
    rospy.spin()

if __name__ == "__main__":
    move_arm_server()