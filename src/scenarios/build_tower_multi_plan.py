import time
import libry as ry
import numpy as np

import networkx as nx
import actions as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_controller
from util.setup_env import setup_tower_env
from feasibility import check_switch_chain_feasibility
from robustness import get_robust_chain, get_robust_set_of_chains
from util.visualize_search import draw_search_graph


"""
Build a tower with the provided plan. 
- During execution, a block is placed to its original position, which should show 
interference in the real world. 
- we want to show that we 
"""


def build_tower(verbose=False, interference=False):

    verbose = True

    # get all actions needed to build a tower
    action_list = [
        con.GrabBlock(),
        con.PlaceOn(),
        con.PlaceSide()
    ]

    # setup config and get frame names
    _, C, block_names = setup_tower_env(3)  # 3 blocks in scene
    gripper_name = "R_gripper"

    # put all objects into one dictionary with types
    scene_objects = {
        dt.type_block: block_names,
        dt.type_gripper: (gripper_name,)
    }

    # get plan and goal
    plan, goal, state_plan, G = get_plan(verbose, action_list, scene_objects)

    # if there is a plan, print it out, otherwise leave
    if plan:
        if verbose:
            print("Found the following plan:")
            for action in plan:
                print(action)
    else:
        print("No plan found!")
        return

    name2con = {x.name: x for x in action_list}  # dict

    grounded_actions = nx.get_edge_attributes(G, "action")
    grounded_ctrlsets = dict()
    for edge in G.edges():
        grounded_action = grounded_actions[edge]
        print(edge, grounded_actions[edge])
        relevant_frames = grounded_action.sig[1:]
        controller = name2con[grounded_action.sig[0]]  # the actual controller

        grounded_ctrlsets[edge] = controller.get_grounded_control_set(C, relevant_frames)

    nx.set_edge_attributes(G, grounded_ctrlsets, "ctrlset")

    # get goal controller, with only immediate conditions features (needed for feasibility)
    goal_controller = get_goal_controller(C, goal)

    draw_search_graph(plan, state_plan, G)

    # get robust tree/ set of chains
    robust_set_of_chains = get_robust_set_of_chains(C, G, state_plan, goal_controller, False)

    robust_plan = robust_set_of_chains[0]

    # check if plan is feasible in current config
    is_feasible, komo_feasy = check_switch_chain_feasibility(C, robust_plan, goal_controller, verbose=False)
    robust_plan = robust_plan[::-1]

    if not is_feasible:
        print("Plan is not feasible in current Scene!")
        print("Aborting")
        return

    # Start simulation of plan here
    C.view()
    tau = .01
    is_done = False

    for name, x in robust_plan:
        pass
        x.add_qControlObjective(2, 1e-5*np.math.sqrt(tau), C)
        x.add_qControlObjective(1, 1e-3*np.math.sqrt(tau), C)
        #x.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e2]), ry.OT.eq)

    # setup for interference
    original_position = C.frame("b2").getPosition()
    original_position[1] = original_position[1]+0.05
    interference_counter = 0
    has_interfered = False

    # simulation loop
    for t in range(0, 10000):
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)
        is_any_controllers_feasible = False

        # check if goal has been reached
        if goal_controller.canBeInitiated(C):
            is_done = True
            break

        # iterate over each controller, check which can be started first
        for i, (name, c) in enumerate(robust_plan):
            if c.canBeInitiated(C):
                ctrl.set(c)
                is_any_controllers_feasible = True
                if i == 1 and interference and not has_interfered:  # 3 works, 1 doesnt
                    interference_counter += 1
                    if interference_counter == 50:
                        block = C.frame("b2")
                        block.setPosition(original_position)
                        has_interfered = True
                if verbose:
                    print(f"Initiating: {name}")
                # leave loop, we have the controller
                break
            else:
                if verbose:
                    print(f"Cannot be initiated: {name}")

        # if no plan is feasible, check other plans
        if not is_any_controllers_feasible:
            print("No controller can be initiated!")
            # lets switch the plan

            for plan in robust_set_of_chains[1:]:
                is_initiated = False
                is_feasible = False
                for i, (name, c) in enumerate(plan[::-1]):
                    if c.canBeInitiated(C):
                        new_plan = plan
                        is_feasible, komo_feasy = check_switch_chain_feasibility(C, new_plan, goal_controller,
                                                                                 verbose=True)
                        is_initiated = True
                        break
                    else:
                        print(f"{name} cannot be initiated!")
                if is_initiated and is_feasible:
                    print("new plan found!")
                    print(plan)
                    robust_plan = plan[::-1]
                    break

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        # coll = C.getCollisions(0)
        time.sleep(tau)

    if is_done:
        print("Plan was finished!")
    else:
        print("Time ran out!")

    time.sleep(10)


if __name__ == '__main__':

    build_tower(verbose=False, interference=True)







