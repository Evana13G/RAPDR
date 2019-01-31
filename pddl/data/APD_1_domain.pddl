(define (domain rapdr)

(:requirements :strips :typing :fluents :disjunctive-preconditions)

(:types
    location obj gripper button - object
    waypoint - location
)

(:predicates
    (gripper_at ?g - gripper ?wp - waypoint)
    (obj_at ?o - obj ?wp - waypoint)
    (button_at ?b - button ?wp - waypoint)
    (pressed ?b - button)
    (is_visible ?o - obj)
)

(:action obtain_object
    :parameters (?g - gripper ?loc0 - waypoint ?o - obj ?loc1 - waypoint )
    :precondition (and
        (gripper_at ?g ?loc0 )
        (obj_at ?o ?loc1 ))
    :effect (and
        (gripper_at ?g ?loc0 )
        (not (obj_at ?o ?loc1 ) ))
)

(:action press_button
    :parameters (?g - gripper ?loc0 - waypoint ?b - button ?loc1 - waypoint )
    :precondition (and
        (gripper_at ?g ?loc0 )
        (button_at ?b ?loc1 ))
    :effect (and
        (gripper_at ?g ?loc0 )
        (pressed ?b ))
)

(:action action0
    :parameters (?g - gripper ?b - button ?0 - obj ?loc0 - waypoint ?loc1 - waypoint ?loc2 - waypoint ?loc3 - waypoint ?loc4 - waypoint ?loc5 - waypoint ?loc6 - waypoint )
    :precondition (and
        (button_at  ?b ?loc4 )
        (obj_at  ?0 ?loc6 )
        (gripper_at  ?g ?loc5 ))
    :effect (and
        (gripper_at  ?g ?loc5 )
        (button_at  ?b ?loc4 )
        (obj_at  ?0 ?loc2 ))
)

(:action action1
    :parameters (?g - gripper ?b - button ?0 - obj ?loc0 - waypoint ?loc1 - waypoint ?loc2 - waypoint ?loc3 - waypoint ?loc4 - waypoint ?loc5 - waypoint ?loc6 - waypoint )
    :precondition (and
        (button_at  ?b ?loc5 )
        (obj_at  ?0 ?loc2 )
        (gripper_at  ?g ?loc6 ))
    :effect (and
        (button_at  ?b ?loc5 )
        (obj_at  ?0 ?loc4 )
        (gripper_at  ?g ?loc6 ))
)

(:action action2
    :parameters (?g - gripper ?b - button ?0 - obj ?loc0 - waypoint ?loc1 - waypoint ?loc2 - waypoint ?loc3 - waypoint ?loc4 - waypoint ?loc5 - waypoint ?loc6 - waypoint )
    :precondition (and
        (button_at  ?b ?loc5 )
        (obj_at  ?0 ?loc4 )
        (gripper_at  ?g ?loc6 ))
    :effect (and
        (button_at  ?b ?loc5 )
        (obj_at  ?0 ?loc2 )
        (gripper_at  ?g ?loc6 ))
)

(:action action3
    :parameters (?g - gripper ?b - button ?0 - obj ?loc0 - waypoint ?loc1 - waypoint ?loc2 - waypoint ?loc3 - waypoint ?loc4 - waypoint ?loc5 - waypoint ?loc6 - waypoint )
    :precondition (and
        (button_at  ?b ?loc4 )
        (obj_at  ?0 ?loc2 )
        (gripper_at  ?g ?loc5 ))
    :effect (and
        (button_at  ?b ?loc4 )
        (obj_at  ?0 ?loc6 )
        (gripper_at  ?g ?loc5 ))
)

(:action action4
    :parameters (?g - gripper ?b - button ?0 - obj ?loc0 - waypoint ?loc1 - waypoint ?loc2 - waypoint ?loc3 - waypoint ?loc4 - waypoint ?loc5 - waypoint ?loc6 - waypoint )
    :precondition (and
        (obj_at  ?0 ?loc2 )
        (gripper_at  ?g ?loc5 )
        (button_at  ?b ?loc4 ))
    :effect (and
        (button_at  ?b ?loc4 )
        (obj_at  ?0 ?loc1 )
        (gripper_at  ?g ?loc5 ))
)

)