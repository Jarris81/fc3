import time
import libry as ry
import numpy as np

import actions as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_controller
from util.setup_env import setup_tower_env
from feasibility import check_switch_chain_feasibility
from robustness import get_robust_system


"""
Build a tower with the provided plan. 
- During execution, a block is placed to its original position, which should show 
interference in the real world. 
- we want to show that we 
"""


def build_tower(verbose=False, interference=False):

    # get all actions needed to build a tower
    actions = [
        con.ApproachBlock(),
        con.PlaceOn(),
        con.CloseGripper(),
        con.OpenGripper()
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
    plan, goal = get_plan(verbose, actions, scene_objects)

    # if there is a plan, print it out, otherwise leave
    if plan:
        if verbose:
            print("Found the following plan:")
            for action in plan:
                print(action)
    else:
        print("No plan found!")
        return

    # convert simple actions to tuple with grounded action and CtrlSet
    controller_tuples = []
    name2con = {x.name: x for x in actions}  # dict
    for grounded_action in plan:
        obj_frames = grounded_action.sig[1:]
        controller = name2con[grounded_action.sig[0]]  # the actual controller
        controller_tuples.append((grounded_action, controller.get_grounded_control_set(C, obj_frames)))

    # get goal controller, with only immediate conditions features (needed for feasibility)
    goal_controller = get_goal_controller(C, goal)

    # add a new controller sequence, which
    controller_tuples_2 = controller_tuples.copy()
    # add some new controller to the new list




    # check if plan is feasible in current config
    is_feasible, komo_feasy = check_switch_chain_feasibility(C, controller_tuples, goal_controller, vis=False, verbose=False)

    # if not is_feasible:
    #     print("Plan is not feasible in current Scene!")
    #     print("Aborting")
    #     return

    # get the robust plan, used in execution
    robust_plan = get_robust_system(C, komo_feasy, controller_tuples, goal_controller)

    # Start simulation of plan here
    C.view()
    tau = .01

    isDone = False

    for name, x in robust_plan:
        pass
        #x.add_qControlObjective(2, 1e-3*np.math.sqrt(tau), C)
        #x.add_qControlObjective(1, 1e-1*np.math.sqrt(tau), C)
        #x.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e2]), ry.OT.eq)

    # create a new action, which only releases the block b1. With this, we can show, that the when interference
    # happens with b2 at i=1 (when holding b1), we can still complete the plan
    new_action = con.OpenGripper()
    new_action_grounded = new_action.get_simple_action(scene_objects).ground("R_gripper", "b1")
    obj_frames = new_action_grounded.sig[1:]
    controller = name2con[new_action_grounded.sig[0]]  # the actual controller
    new_controller = controller.get_grounded_control_set(C, obj_frames)

    # add to end of the robust plan
    robust_plan.append((new_action_grounded, new_controller))

    # simulation loop

    # setup for interference
    original_position = C.frame("b2").getPosition()
    original_position[1] = original_position[1]+0.05
    interference_counter = 0
    has_interfered = False



    for t in range(0, 10000):
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        # check if goal has been reached
        if goal_controller.canBeInitiated(C):
            isDone = True
            break

        #
        is_any_controllers_feasible = False

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

        if not is_any_controllers_feasible and verbose:
            print("No controller can be initiated!")

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        coll = C.getCollisions(0)
        time.sleep(tau)

    if isDone:
        print("Plan was finished!")
    else:
        print("Time ran out!")

    time.sleep(10)


if __name__ == '__main__':

    build_tower(verbose=False, interference=True)







