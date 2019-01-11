#!/usr/bin/env python


###### put a goal 
###### get a pddl plan for it 
###### try to execute it
###### if fails, get predicates list, and launch APV for each action until success
###### using the change points, test the actions and fire off the predicates list, storing all 
###### try each of the actions, return if successful
######analyze the new actions


import rospy

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *
from util.knowledge_base import KnowledgeBase

from std_msgs.msg import (
    Header,
    Empty,
)

def main():
    rospy.init_node("agent_brain")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.wait_for_service('plan_generator_srv', timeout=60)
    # rospy.wait_for_service('plan_executor_srv', timeout=60)
    rospy.wait_for_service('APV_srv', timeout=60)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('scenario_data_srv', timeout=60)

    KB = KnowledgeBase()

    try:

        goal = "(:goal (and (object_at block loc0b) (pressed left_button)))"
        
        data = KB.getDomainData()

        taskName = data['domain']
        types = data['types']
        predicates = data['predicates']
        requirements = data['requirements']
        actions = data['actions']

        planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
        
        resp = planGenerator(Domain(taskName, types, predicates, requirements, actions))
        print(resp)
        # planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        
        APV = rospy.ServiceProxy('APV_srv', APVSrv)
        partialActionExecutor = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)


    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    main()
