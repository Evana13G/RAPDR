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
from util.data_conversion import * 

import random
from std_msgs.msg import (
    Header,
    Empty,
)

def main():
    rospy.init_node("agent_brain")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('APV_srv', timeout=60)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('scenario_data_srv', timeout=60)
    rospy.wait_for_service('plan_generator_srv', timeout=60)
    rospy.wait_for_service('plan_executor_srv', timeout=60)

    KB = KnowledgeBase()

    try:

        # Services
        planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
        planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        APV = rospy.ServiceProxy('APV_srv', APVSrv)
        partialActionExecutor = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)


        #####################################################################################
        #                                                                                   #
        #    This is where the highest level of the main algorithm should be implemented    #
        #                                                                                   #
        #    Domain:                     Problem:                                           #
        #    string name                 string task                                        #
        #    string[] requirements       string domain                                      #
        #    string[] types              string[] objects                                   #
        #    string[] predicates         string[] init                                      #
        #    string[] actions            string goal                                        #
        #                                                                                   #
        #####################################################################################
        attempt = 1

        domainDict = KB.getDomainData()
        domainName = domainDict['domain']
        types = domainDict['types']
        predicates = domainDict['predicates']
        requirements = domainDict['requirements']
        actions = domainDict['actions']

        task = 'APD'
        currentState = scenarioData()
        objs = currentState.objects
        init = currentState.init

        goal = ['(pressed left_button)']
        domain = Domain(domainName, requirements, types, predicates, actions)
        problem = Problem(task, domainName, objs, init, goal)
        filename = task + '_' + str(attempt)

        plan = planGenerator(domain, problem, filename)
        # executionSuccess = planExecutor(plan.plan)
        executionSuccess = 0
        if executionSuccess != 1:

            momentOfFailurePreds = scenarioData().predicates
            print("Preds at moment of failure: ")
            print(momentOfFailurePreds)
            # Should prob break this into a diff module... findNewAction module 
            # Do one that can for each action, return new sub actoins to try ?

            APVtrials = []
            objectsToIterate = pddlObjects(currentState.predicateList)
            
            for action in KB.getActions():
                args = action.getNonLocationVars()
                actionTrials = []
                actionTrials.append(action.getName())
                for arg in args:
                    itemsChoices = objectsToIterate[arg]
                    choice = itemsChoices[random.randint(0, len(itemsChoices) - 1)]
                    actionTrials.append(choice)
                if len(args) < 4:
                    actionTrials.append(None)
                APVtrials.append(actionTrials)

            trialNo = 0
            while(trialNo < len(APVtrials)):
                print("..Trying APV...")

                print(APVtrials[trialNo])
                try:
                    resp = APV(APVtrials[trialNo][0], APVtrials[trialNo][1], APVtrials[trialNo][2], APVtrials[trialNo][3])
                    print(str(len(resp.endEffectorInfo)) + " total change points found")

                    print("..Trying Partial Plan Executor...")
                    i = 0
                    while i <= len(resp.endEffectorInfo) - 2:
                        print("Starting iteration " + str(i+1))


                        startingState = scenarioData().predicates

                        resp_2 = partialActionExecutor(resp.endEffectorInfo[i], resp.endEffectorInfo[i+1])
                        print("Success bool for iteration " + str(i+1) + ": " + str(resp_2.success_bool))
                        
                        endingState = scenarioData().predicates

                        print("Starting preds: ")
                        print(startingState)
                        print("Ending preds :")
                        print(endingState)

                        i = i + 1 
                except rospy.ServiceException, e:
                    print("Service call failed: %s"%e)
                trialNo = trialNo + 1 
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    main()
