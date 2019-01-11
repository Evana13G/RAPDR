(define (problem task)
(:domain rapdr1)
(:objects
    loc0a loc0b loc0 loc1 loc2 loc3 - waypoint
    left_button right_button - button
    block - obj
    left right - gripper)

(:init
    (gripper_at left loc0a)
    (gripper_at right loc0b)
    (button_at left_button loc1)
    (button_at right_button loc2)
    (object_at block loc3))

(:goal (and (object_at block loc0b) (pressed left_button)))

)
