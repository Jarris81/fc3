import libry as ry
from planners import get_tower_goal_controller
from util.setup_env import setup_tower_env
import predicates as pred
import util.constants as dt


_, C, block_names = setup_tower_env(3)  # 3 blocks in scene

gripper_name = "R_gripper"

# put all objects into one dictionary with types
scene_obj = {
    dt.type_block: block_names,
    dt.type_gripper: (gripper_name,)
}

# normal goal predicates
goal = list()

# hand should be free
goal_hand_empty = pred.HandEmpty("G")
goal_hand_empty.ground_predicate(G=scene_obj[dt.type_gripper][0])
goal.append(goal_hand_empty.get_grounded_predicate())

# blocks on blocks
for i in range(len(scene_obj[dt.type_block]) - 1):
    block_on_block = pred.BlockOnBlock("B", "B_placed")
    block_on_block.ground_predicate(B=scene_obj[dt.type_block][i], B_placed=scene_obj[dt.type_block][i+1])
    goal.append(block_on_block.get_grounded_predicate())

print(goal)

goal_controller = get_tower_goal_controller(C, goal)

C_copy = ry.Config()
C_copy.copy(C)

# check feasibility with komo switches
komo = ry.
komo.setModel(C_copy, True)  # use swift collision engine
komo.setTiming(1, 1, 5., 1)

komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
komo.addSquaredQuaternionNorms([], 3.)

for o in goal_controller.getObjectives():
    if o.get_OT() == ry.OT.sos:
        f = o.feat()
        komo.addObjective([1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
