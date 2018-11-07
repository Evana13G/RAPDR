(define (problem task)
(:domain rapdr)
(:objects
    loc0a loc0b loc0 loc1 loc2 loc3 - waypoint
    button1 button2 - button
    block3 - object
    robot - agent
    right_gripper left_gripper - gripper
)
(:init
    (gripper_at left_gripper loc0a)
    (gripper_at right_gripper loc0b)
    (button_at button1 loc1)
    (button_at button2 loc2)
    (object_at block3 loc3)
    (open left_gripper)
    (open right_gripper)
)
(:goal (closed left_gripper))

)
