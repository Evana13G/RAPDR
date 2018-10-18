(define (problem task)
(:domain rapdr)
(:objects
    loc0 loc1 loc2 loc3 - waypoint
    button1 button2  - button
    block3 - block
    robot - agent
    right_gripper left_gripper - gripper
)
(:init
    (gripper_at left_gripper loc0)
    (block_at block3 loc3)
)
(:goal (block_at block3 loc0)))
