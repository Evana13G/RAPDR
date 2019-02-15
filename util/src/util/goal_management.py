from util.physical_agent import PhysicalAgent 

def goalAccomplished(goalList, currentState):
	numGoalsAccomplished = 0
	for goal in goalList:
		if goal in currentState:
			numGoalsAccomplished = numGoalsAccomplished + 1
	if numGoalsAccomplished == len(goalList):
		return True
	return False


# def findAllCombos(objects, action):
def isViable(action):
    if action.getEffects() == []:
        return False
    return True

def moveLeftArmToStart(lPa):
    starting_joint_angles_l = {'left_w0': 0.6699952259595108,
                               'left_w1': 1.030009435085784,
                               'left_w2': -0.4999997247485215,
                               'left_e0': -1.189968899785275,
                               'left_e1': 1.9400238130755056,
                               'left_s0': -0.08000397926829805,
                               'left_s1': -0.9999781166910306}
    lPa.move_to_start(starting_joint_angles_l)

def moveRightArmToStart(rPa):
    starting_joint_angles_r = {'right_e0': -0.39888044530362166,
                                'right_e1': 1.9341522973651006,
                                'right_s0': 0.936293285623961,
                                'right_s1': -0.9939970420424453,
                                'right_w0': 0.27417171168213983,
                                'right_w1': 0.8298780975195674,
                                'right_w2': -0.5085333554167599}
    rPa.move_to_start(starting_joint_angles_r)

