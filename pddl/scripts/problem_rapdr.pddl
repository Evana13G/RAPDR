(define (problem task)
(:domain rapdr)
(:objects
    loc0a loc0b loc0 loc1 loc2 loc3 - waypoint
    button1 button2 - button
    block3 - object
    robot - agent
    right left - gripper
)
(:init
    (gripper_at left loc0a)
    (gripper_at right loc0b)
    (button_at button1 loc1)
    (button_at button2 loc2)
    (object_at block3 loc3)
)
(:goal (object_at block3 loc0a)))
