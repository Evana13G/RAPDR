#!/usr/bin/env python


from agent.srv import *
from std_msgs.msg import (
    Empty,
)
import rospy


def main():
    rospy.init_node("agent_test_node")

    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.wait_for_service('APV_srv', timeout=60)
    # rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    # rospy.wait_for_service('scenario_data_srv', timeout=60)
    # rospy.wait_for_service('plan_generator_srv', timeout=60)
    # rospy.wait_for_service('plan_executor_srv', timeout=60)
    # rospy.wait_for_service('press_button_srv', timeout=60)
    # rospy.wait_for_service('obtain_object_srv', timeout=60)

    rospy.wait_for_service('brain_A_srv', timeout=60)
    rospy.wait_for_service('brain_B_srv', timeout=60)


    try:
        brain_A = rospy.ServiceProxy('brain_A_srv', BrainSrv)
        brain_B = rospy.ServiceProxy('brain_B_srv', BrainSrv)
    
        print("..Trying Brain A...")
        resp_A = brain_A('testA', 100, 10)
        print("Success bool for brain A: " + str(resp_A.success_bool))

        print("..Trying Brain A...")
        resp_B = brain_B('testB', 100, 10)
        print("Success bool for brain B: " + str(resp_B.success_bool))

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
