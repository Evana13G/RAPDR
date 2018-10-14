(define (domain rapdr)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	agent
	block
	gripper
	button
)

(:predicates
	(gripper_at ?g - gripper ?wp - waypoint)
	(block_at ?o - block ?wp - waypoint)
	(button_at ?b - button)
	(pressed ?b -button)
	(open ?g - gripper)
)

;; Open gripper
(:durative-action open_gripper
	:parameters (?g - gripper)
	:duration ( = ?duration 60)
	:condition (at start (gripper_at ?g ?wp))
	:effect (at end (open ?g))
)

;; Close gripper 
(:durative-action close_gripper
	:parameters (?g - gripper)
	:duration ( = ?duration 60)
	:condition (and
		(at start (open ?g))
		(at start (gripper_at ?g ?wp)))
	:effect (at start (not (open ?g)))
)
)

;; Push a button with gripper
;; (:durative-action push_button)

;; Move gripper to waypoint where object is located
;; (:durative-action move_gripper_to_object))
