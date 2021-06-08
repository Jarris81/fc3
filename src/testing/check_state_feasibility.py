import libry as ry
import actions
import predicates
import util.constants as dt
import time
from util.setup_env import setup_tower_env

action_list = [
        actions.GrabBlock(),
        actions.PlaceOn(),
    ]

objects = {
         dt.type_block: [f"b{x+1}" for x in range(3)],  # , "b3"),
         dt.type_gripper: ("R_gripper",)
     }

all_frames = [x for y in objects.values() for x in y]

print(f"{all_frames=}")

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
freeblock.ground_predicate(B="b2")



#print(freeblock.features(C, all_frames))
#print(freeblock.is_feasible(C, all_frames))
#print(inhand.features(C, all_frames)[0].eval(C))


C.view()

a = actions.GrabBlock()
x = a.get_grounded_control_set(C, ["R_gripper", "b1"], ["b2", "b3"])[0]


ctrl_set = ry.CtrlSet()
for feature in freeblock.features(C, all_frames):
    ctrl_set.addObjective(feature, ry.OT.ineq, -1)

for symcommand in handempty.symbolic_commands(C, all_frames):
    ctrl_set.addSymbolicCommand(*symcommand)

tau = .01
ctrl = ry.CtrlSolver(C, tau, 2)
print("block free")
print("can be init:", x.canBeInitiated(C))

for t in range(0, 10000):
    # create a new solver everytime
    ctrl = ry.CtrlSolver(C, tau, 2)

    if x.canBeInitiated(C):
        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        coll = C.getCollisions(0)
        time.sleep(tau)



#print(C.getCollisions())

# check if the predicate is feasible

# for x in preds:
#
#     print(f"{x.name} is feasible: {x.is_feasible(C, all_frames)} for {x.get_grounded_predicate()}")

time.sleep(5)
