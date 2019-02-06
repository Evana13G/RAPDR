#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from agent.msg import PressButtonActionAction, PressButtonActionGoal

if __name__ == '__main__':
    rospy.init_node('press_button_action_client') # Check that node name = file name doesn't affect anything
    client = actionlib.SimpleActionClient('press_button_action', PressButtonActionAction)
    client.wait_for_server()

    goal = PressButtonActionGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))