(define (domain rapdr)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	agent
	object
	gripper
	button
)

(:predicates
	(gripper_at ?g - gripper ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(button_at ?b - button ?wp - waypoint)
	(pressed ?b -button)
	(open ?g - gripper)
	(closed ?g - gripper)
	(object_grasped ?g - gripper ?o - object)
)

;; Open gripper
(:durative-action open_gripper
	:parameters (?g - gripper)
	:duration ( = ?duration 60)
	:condition (at start (closed ?g))
	:effect (and
		(at start (not (closed ?g)))
		(at end (open ?g)))
)

;; Close gripper 
(:durative-action close_gripper
	:parameters (?g - gripper)
	:duration ( = ?duration 60)
	:condition (at start (open ?g))
	:effect (and 
		(at start (not (open ?g)))
		(at end (closed ?g)))
)

;; Move gripper to waypoint where object is located
(:durative-action move_gripper_to_object
	:parameters (?g - gripper ?wp0 - waypoint ?o - object ?wp1 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (gripper_at ?g ?wp0))
		(over all (object_at ?o ?wp1)))
	:effect (and
		(at end (gripper_at ?g ?wp1))
		(at start (not (gripper_at ?g ?wp0))))
)


;; Move gripper to waypoint where object is located
(:durative-action move_gripper_to_button
	:parameters (?g - gripper ?wp0 - waypoint ?b - button ?wp1 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (gripper_at ?g ?wp0))
		(over all (button_at ?b ?wp1)))
	:effect (and
		(at end (gripper_at ?g ?wp1))
		(at start (not (gripper_at ?g ?wp0))))
)

(:durative-action move_object
	:parameters (?g - gripper ?o - object ?wp0 - waypoint ?wp1 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (gripper_at ?g ?wp0))
		(at start (object_at ?o ?wp0))
		(over all (object_grasped ?g ?o)))
	:effect (and
		(at end (gripper_at ?g ?wp1))
		(at end (object_at ?o ?wp1))
		(at start (not (gripper_at ?g ?wp0)))
		(at start (not (object_at ?o ?wp0))))
)


;; Grasp with gripper
(:durative-action grasp_object
	:parameters (?g - gripper ?o - object ?wp0 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(over all (gripper_at ?g ?wp0))
		(over all (object_at ?o ?wp0)))
	:effect (at end (object_grasped ?g ?o))
)

;; Push a button with gripper
(:durative-action push_button
	:parameters (?g - gripper ?b - button ?wp0 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(over all (gripper_at ?g ?wp0))
		(over all (button_at ?b ?wp0)))
	:effect (at end (pressed ?b))
)

)

;; Can do 'over all' predicates