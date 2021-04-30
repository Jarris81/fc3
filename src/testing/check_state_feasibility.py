import graphviz
import testing.tower_planner as planner
import actions
import predicates
import util.domain_tower as dt
import time
from util.setup_env import setup_tower_env

action_list = [
        actions.ApproachBlock(),
        actions.PlaceOn(),
    ]

objects = {
         dt.type_block: (1, 2, 3),  # , "b3"),
         dt.type_gripper: ("gripper_R",)
     }

# Parse arguments
#plan = planner.get_plan(True, action_list, objects)
#print(plan)

inhand = predicates.InHand("G", "B1")
collisionfree = predicates.CollisionFree()
bonb = predicates.BlockOnBlock("B", "B_placed_on")
handempty = predicates.HandEmpty("G")
freeblock = predicates.BlockFree("B")

preds = [
    inhand,
    collisionfree,
    bonb,
    handempty,
    freeblock
]

_, C, block_names = setup_tower_env(3)  # 3 blocks in scene

inhand.ground_predicate(G="R_gripper", B1="b1")
bonb.ground_predicate(B="b1", B_placed_on="b2")
collisionfree.ground_predicate()
handempty.ground_predicate(G="R_gripper")
freeblock.ground_predicate(B="b1")

print(handempty.features(C))
print(inhand.features(C)[0].eval(C))


C.view()

# check if the predicate is feasible

for x in preds:

    print(f"{x.name} is feasible: {x.is_feasible(C, *block_names)} for {x.get_grounded_predicate()}")

time.sleep(5)
