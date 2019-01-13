(define (problem  APD)

(:domain rapdr)

(:objects
    1.0,-0.0,-0.0 0.54,0.15,0.04 0.53,0.15,-0.27 0.53,-0.27,-0.27 0.79,0.01,-0.29 0.56,-0.33,-0.04 - waypoint
    right_button left_button - button
    left_gripper right_gripper - gripper
    table wall block - obj
)

(:init
    (gripper_at left_gripper 0.54,0.15,0.04)
    (gripper_at right_gripper 0.56,-0.33,-0.04)
    (object_at table 1.0,-0.0,-0.0)
    (object_at wall 1.0,-0.0,-0.0)
    (object_at block 0.79,0.01,-0.29)
    (button_at right_button 0.53,-0.27,-0.27)
    (button_at left_button 0.53,0.15,-0.27)
)

(:goal (pressed left_button))

)