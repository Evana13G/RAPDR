#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from agent.msg import PressButtonActionAction

class PressButtonActionServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('press_button_action', PressButtonActionAction, self.execute, False)
    self.server.start()

  def execute(self, goal):  
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('press_button_action_server')
  server = PressButtonActionServer()
  rospy.spin()