from pyddl import Domain, Problem, State, Action, neg, backwards_planner
import util.domain_tower as dt
import actions as con

actions = [
        con.ApproachBlock(),
        con.PlaceOn(),
        con.CloseGripper(),
        con.OpenGripper()
    ]

# setup config and get frame names
block_names = [f"b{x}" for x in range(3)]
gripper_name = "R_gripper"

# put all objects into one dictionary with types
scene_obj = {
    dt.type_block: block_names,
    dt.type_gripper: (gripper_name,)
}

# get simple action from all controllers
domain = Domain((x.get_simple_action(scene_obj) for x in actions))

# goal is for now numerical order of block placed on each other, unless specified otherwise
goal = [(dt.b_on_b, scene_obj[dt.type_block][i], scene_obj[dt.type_block][i + 1])\
        for i in range(len(scene_obj[dt.type_block]) - 1)]
# also append free hand
goal.append((dt.hand_empty, scene_obj[dt.type_gripper][0]))

# normal initial conditions
init_free_hand = (dt.hand_empty, scene_obj[dt.type_gripper][0])
init_free_blocks = [(dt.block_free, block) for block in scene_obj[dt.type_block]]

# extend all initial conditions
init = [init_free_hand]
init.extend(init_free_blocks)

# define problem here
prob = Problem(
    domain,
    scene_obj,
    init=init,
    goal=goal
)

plan = backwards_planner(prob, state0=init, goal=goal)

