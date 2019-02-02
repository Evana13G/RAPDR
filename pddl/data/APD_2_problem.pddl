(define (problem  APD)

(:domain rapdr)

(:objects
    0.6,0.2,0.1  0.6,-0.3,0.0  0.8,0.0,-0.3  1.0,0.0,0.0  0.5,-0.3,-0.3  0.5,0.2,-0.3  - waypoint
    right_button left_button - button
    left_gripper right_gripper - gripper
    wall table block - obj
)

(:init
    (button_at left_button 0.5,0.2,-0.3)
    (gripper_at left_gripper 0.6,0.2,0.1)
    (gripper_at right_gripper 0.6,-0.3,0.0)
    (obj_at table 1.0,0.0,0.0)
    (obj_at wall 1.0,0.0,0.0)
    (obj_at block 0.8,0.0,-0.3)
    (button_at right_button 0.5,-0.3,-0.3)
)

(:goal (is_visible block))

)