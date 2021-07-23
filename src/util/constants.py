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

