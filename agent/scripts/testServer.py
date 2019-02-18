#!/usr/bin/env python


from agent.srv import *
from std_msgs.msg import (
    Empty,
)
import rospy
from environment.srv import HandleEnvironmentSrv




def main():
    rospy.init_node("agent_test_node")

    rospy.wait_for_message("/robot/sim/started", Empty)
    rospy.wait_for_service('brain_A_srv', timeout=60)
    # rospy.wait_for_service('brain_B_srv', timeout=60)
    rospy.wait_for_service('init_environment', timeout=60)

    env = rospy.ServiceProxy('init_environment', HandleEnvironmentSrv)


    cluster1_trials = 2
    cluster2_trials = 5

    try:
        brain_A = rospy.ServiceProxy('brain_A_srv', BrainSrv)
        # brain_B = rospy.ServiceProxy('brain_B_srv', BrainSrv)
        
        print("********************************")
        print("Running CLUSTER 1 EXPERIMENTS\n")
        for i in range(cluster1_trials):
            print("Trial # " + str(i) + ', Brain A')
            testName = 'TEST_' + str(i)
            resp_A = brain_A(testName, 100, 10)

            print(resp_A.timePerAttempt)
            print(resp_A.totalTime)

            env('restart')
        

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
