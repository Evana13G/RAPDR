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
	(pressed ?b - button)
	(is_visible ?o - object)
)

;; Grab an object and bring it back to orig location
(:durative-action obtain_object
	:parameters (?g - gripper ?loc0 - waypoint ?o - object ?loc1 -waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (gripper_at ?g ?loc0))
		(at start (object_at ?o ?loc1)))
	:effect (and
		(at end (object_at ?o ?loc0))
		(at start (not (object_at ?o ?loc1))))
)


;; Press a button with gripper
(:durative-action press_button
	:parameters (?g - gripper ?loc0 -waypoint ?b - button ?loc1 - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (gripper_at ?g ?loc0))
		(over all (button_at ?b ?loc1)))
	:effect (at end (gripper_at ?g ?loc0))
)

)
