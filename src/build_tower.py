import time
import libry as ry
import numpy as np

import controllers as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_controller
from util.setup_env import setup_tower_env
from feasibility import check_feasibility
from robustness import  get_robust_system


"""
Build a tower with the provided plan
"""


def build_tower(verbose=False):

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
    name2con = {x.name: x for x in actions} #  dict
    for grounded_action in plan:
        obj_frames = grounded_action.sig[1:]
        controller = name2con[grounded_action.sig[0]]  # the actual controller
        controller_tuples.append((grounded_action, controller.get_grounded_control_set(C, obj_frames)))

    # get goal controller, with only immediate conditions features (needed for feasibility)
    goal_controller = get_goal_controller(C, goal)

    # check if plan is feasible in current config
    komo_feasy = check_feasibility(C, controller_tuples, steps_per_keyframe=1, vis=False, goal=goal_controller)

    # get the robust plan, used in execution
    robust_plan = get_robust_system(C, komo_feasy, controller_tuples, goal_controller)

    # Start simulation of plan here
    C.view()
    tau = .01

    isDone = False

    # simulation loop
    for t in range(0, 10000):

        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        # check if goal has been reached
        if goal_controller.canBeInitiated(C):
            isDone = True
            break

        # iterate over each controller, check which can be started first
        for name, c in robust_plan:
            if c.canBeInitiated(C):
                ctrl.set(c)
                if verbose:
                    print(f"Initiating: {name}")
                break
            else:
                if verbose:
                    print(f"Cannot be initiated: {name}")

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        time.sleep(tau)

    if isDone:
        print("Plan was finished!")
    else:
        print("time ran out!")

    time.sleep(10)


if __name__ == '__main__':

    build_tower()







