(define (domain rapdr1)

(:requirements :strips :typing :fluents :disjunctive-preconditions)

(:types
	location obj gripper button - object
	waypoint - location
)

(:predicates
	(gripper_at ?g - gripper ?wp - waypoint)
	(object_at ?o - obj ?wp - waypoint)
	(button_at ?b - button ?wp - waypoint)
	(pressed ?b - button)
	(is_visible ?o - obj)
)

;; Grab an object and bring it back to orig location
(:action obtain_object
	:parameters (?g - gripper ?loc0 - waypoint ?o - obj ?loc1 - waypoint)
	:precondition (and
		(gripper_at ?g ?loc0)
		(object_at ?o ?loc1))
	:effect (and
		(object_at ?o ?loc0)
		(not (object_at ?o ?loc1)))
)


;; Press a button with gripper
(:action press_button
	:parameters (?g - gripper ?loc0 - waypoint ?b - button ?loc1 - waypoint)
	:precondition (and
		(gripper_at ?g ?loc0)
		(button_at ?b ?loc1))
	:effect (and
		(gripper_at ?g ?loc0)
		(pressed ?b))
)

)
