import pyddl
import actions as con

import util.constants as dt

# test, by seing if right plan comes. Basically, we need to invert actions

gripper_name = "R_gripper"
block_1 = "b1"

# put all objects into one dictionary with types
scene_objects = {
    dt.type_block: (block_1,),
    dt.type_gripper: (gripper_name,)
}

goal = [(dt.in_hand, gripper_name, block_1)]



action = con.OpenGripper()



