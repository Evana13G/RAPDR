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
)

;; Open gripper
(:durative-action open_gripper
	:parameters (?g - gripper ?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (at start (gripper_at ?g ?wp))
	:effect (at end (open ?g))
)

;; Close gripper 
(:durative-action close_gripper
	:parameters (?g - gripper ?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (open ?g))
		(at start (gripper_at ?g ?wp)))
	:effect (at start (not (open ?g)))
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