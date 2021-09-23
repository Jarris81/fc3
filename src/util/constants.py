import numpy as np

"""
Logic space for planner
"""
# Object types
type_gripper = "gripper"
type_block = "block"
type_stick = "stick"
type_bottle = "bottle"

# logic states predicates
in_hand = "in_hand"  # a block is in hand (block, gripper)
block_free = "block_free"  # condition a block is graspable
hand_empty = "empty"  # condition a hand is free
b_on_b = "block_on_block"  # condition block is on block


tower_b1_pos_xy = (0.7, 0.4)
tower_b2_pos_xy = (0.4, 0.3)
tower_b3_pos_xy = (0.6, 0.2)

handover_b1_pos_xy = (0.5, 0.4)

goal_handover_block_pos = (-0.8, 0.3, 0.63)
goal_stick_pull_block_pos = (0.8, 0.1, 0.63)
goal_block_pos = None


table_height = 0.62

l_gripper_name = "l_gripper"
r_gripper_name = "r_gripper"

