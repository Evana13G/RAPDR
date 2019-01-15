(define (problem  APD)

(:domain rapdr)

(:objects
    1.0,-0.0,0.0 0.81,0.02,-0.28 0.53,-0.27,-0.27 1.0,-0.0,-0.0 0.6,0.04,0.01 0.53,0.15,-0.27 0.56,-0.33,-0.04 - waypoint
    right_button left_button - button
    left_gripper right_gripper - gripper
    wall table block - obj
)

(:init
    (object_at table 1.0,-0.0,-0.0)
    (object_at wall 1.0,-0.0,0.0)
    (object_at block 0.81,0.02,-0.28)
    (button_at right_button 0.53,-0.27,-0.27)
    (button_at left_button 0.53,0.15,-0.27)
    (gripper_at left_gripper 0.6,0.04,0.01)
    (gripper_at right_gripper 0.56,-0.33,-0.04)
)

(:goal (is_visible block))

)