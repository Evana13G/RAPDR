#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from gazebo_msgs.msg import LinkStates

class Scenario(object):
    def __init__(self):
        self.sub = rospy.Subscriber("gazebo/joint_state", LinkStates, self.callback)
        self.pub = rospy.Publisher("gazebo_info", String, queue_size=10) # defines the talkers interface to teh rest of ros 

    def scenario():
        while not rospy.is_shutdown():
            self.pub.publish("Five")
            # rate.sleep() # Every time a state chagne message is pubished from the ObtainObject

    def callback(data):
        print("Callback function invoked in scenario data")
        self.pub.publish("Five")
        # print(data)
        # if(data):
        #     pub_data = "True"

def main():
    rospy.init_node('scenario', anonymous=True)
    scenario = Scenario()
    # rate = rospy.Rate(10) # 10hz
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass