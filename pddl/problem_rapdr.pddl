(define (problem task)
(:domain rapdr)
(:objects
    loc0 loc1 loc2 loc3 - waypoint
    button1 button2 - button
    block3 - object
    robot - agent
    right_gripper left_gripper - gripper
)
(:init
    (gripper_at left_gripper loc0)
    (button_at button1 loc1)
    (button_at button2 loc2)
    (object_at block3 loc3)
)
(:goal (pressed button1))

)
