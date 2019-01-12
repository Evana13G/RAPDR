(define (problem  APD)

(:domain rapdr)

(:objects
    (0.57, 0.17, 0.08) (0.56, -0.35, -0.04) (1.0, -0.0, -0.0) (1.0, -0.0, 0.0) (0.8, 0.02, -0.29) (0.53, -0.27, -0.27) (0.53, 0.15, -0.27) - waypoint
    right_button left_button - buttons
    left_gripper right_gripper - grippers
    table wall block - objects
)

(:init
    (gripper_at left_gripper (0.57, 0.17, 0.08))
    (gripper_at right_gripper (0.56, -0.35, -0.04))
    (object_at table (1.0, -0.0, -0.0))
    (object_at wall (1.0, -0.0, 0.0))
    (object_at block (0.8, 0.02, -0.29))
    (button_at right_button (0.53, -0.27, -0.27))
    (button_at left_button (0.53, 0.15, -0.27))
)

(:goal (and 
    (object_at block loc0b)
    (pressed left_button))
)

)