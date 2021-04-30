import sys
import time
import libry as ry


import actions
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_controller
from util.setup_env import setup_tower_env
from feasibility import check_switch_chain_feasibility
from robustness import get_robust_system


"""
Build a tower with the provided plan
"""


def build_tower(verbose=False, interference=False):

    # get all actions needed to build a tower
    action_list = [
        actions.ApproachBlock(),
        actions.PlaceOn(),
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
    plan, goal = get_plan(verbose, action_list, scene_objects)

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
    name2con = {x.name: x for x in action_list}  # dict
    for j, grounded_action in enumerate(plan):
        obj_frames = grounded_action.sig[1:]
        action = name2con[grounded_action.sig[0]]  # the actual controller
        # if j == 1 or j == 5:
        #     continue
        for i, controller in enumerate(action.get_grounded_control_set(C, obj_frames)):
            controller_tuples.append((f"{action.name}_{i}", controller))

    # get goal controller, with only immediate conditions features (needed for feasibility)
    goal_controller = get_goal_controller(C, goal)

    # check if plan is feasible in current config
    is_feasible, komo_feasy = check_switch_chain_feasibility(C, controller_tuples, goal_controller, verbose=False)


    if not is_feasible:
        print("Plan is not feasible in current Scene!")
        print("Aborting")
        return

    # get the robust plan, used in execution
    robust_plan = get_robust_system(C, komo_feasy, controller_tuples, goal_controller, verbose=verbose)

    #return

    for name, x in robust_plan:
        pass  # TODO when adding control objectives to Placing the object on other block, for some reason it does not\
        # TODO converge and therefore Opengripper can not be initiated
        #x.add_qControlObjective(2, 1e-3*np.math.sqrt(tau), C)
        #x.add_qControlObjective(1, 1e-1*np.math.sqrt(tau), C)
        #x.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e2]), ry.OT.eq)

    # Start simulation of plan here
    C.view()

    # simulation variables
    all_controllers_unfeasible = True
    is_done = False
    tau = .01

    # setup for interference
    ori = C.frame("b2").getPosition()
    ori[1] = ori[1] + 0.05 # change the position a little
    interference_counter = 0
    # say where we want intereference
    has_interfered = {1: False, 3: False}

    # simulation loop
    for t in range(0, 10000):

        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        # check if goal has been reached
        if goal_controller.canBeInitiated(C):
            is_done = True
            break

        # reset, to check if at least one controller can be initiated
        all_controllers_unfeasible = True

        # iterate over each controller, check which can be started first
        for i, (name, c) in enumerate(robust_plan):
            if c.canBeInitiated(C):
                ctrl.set(c)
                all_controllers_unfeasible = False
                if interference and i in has_interfered and not has_interfered[i]:  # 3 works, 1 doesnt
                    interference_counter += 1
                    if interference_counter == 50:
                        block = C.frame("b2")
                        block.setPosition(ori)
                        has_interfered[i] = True
                        interference_counter = 0
                if verbose:
                    print(f"Initiating: {name}")
                break
            else:
                if verbose:
                    print(f"Cannot be initiated: {name}")

        if all_controllers_unfeasible and verbose:
            print("No controller can be initiated!")

        # update simulation/ make a step
        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        coll = C.getCollisions(0)
        time.sleep(tau)

    if is_done:
        print("Plan was finished!")
    else:
        print("time ran out!")

    time.sleep(10)


if __name__ == '__main__':

    add_interference = False
    add_verbose = False

    if "verbose" in sys.argv:
        add_verbose = True
    if "interference" in sys.argv:
        add_interference = True

    build_tower(verbose=add_verbose, interference=add_interference)







