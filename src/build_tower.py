import time
import libry as ry
import numpy as np

import controllers as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_feature
from util.setup_env import setup_tower_env
from feasibility import check_feasibility
from feasibility import check_feasibility2


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

    name2con = {x.name: x for x in actions}

    # setup config and get frame names
    _, C, block_names = setup_tower_env(3)
    gripper_name = "R_gripper"

    # put all objects into one dictionary with types
    scene_objects = {
        dt.type_block: block_names,  # , "b3"),
        dt.type_gripper: (gripper_name,)
    }
    # get plan and goal
    plan, goal = get_plan(verbose, actions, scene_objects)

    # if there is a plan, print it out, otherwise leave
    if plan:
        if verbose:
            print("Found the following plan:")
            for ctrlset in plan:
                print(ctrlset)
    else:
        print("No plan found!")
        return

    # convert simple actions to tuple with grounded action and CtrlSet
    control_sets = []
    for grounded_action in plan:
        obj_frames = grounded_action.sig[1:]
        controller = name2con[grounded_action.sig[0]]  # the actual control
        control_sets.append((grounded_action, controller.get_grounded_control_set(C, obj_frames)))

    # check if plan is feasible in current config
    komo_feasy = check_feasibility2(C, control_sets, steps_per_keyframe=1, hack=False, vis=True)

    # get goal condition
    goal_feature = get_goal_feature(C, goal)
    # get implicit conditions
    robust_plan = []



    # not the way to go, should use the C and komo to check if feature is needed
    def is_equal_feature(f1, f2, C):
        if not f1.getFS() == f2.getFS():
            return False

        if not f1.getFrameNames(C) == f2.getFrameNames(C):
            return False

        if not np.all(np.isclose(f1.getTarget(), f2.getTarget())):
            return False

        if not np.all(np.isclose(f1.getScale(), f2.getScale())):
            return False

        return True

    # Better: use the feasibility check, to see if the feature was already fulfilled in previous enter
    # if feature is fulfilled, it is a immediate feature, otherwise it is transient feature
    def is_feature_fulfilled(curr_trans=None, imm_next=None):

        Ccopy = ry.Config()
        Ccopy.copy(C)
        frames_state = 0 # needs to be initialized somehow
        frames_state = komo_feasy.getPathFrames(frames_state)
        print(frames_state)

        Ccopy.setFrameState(frames_state[-3])
        Ccopy.view()
        time.sleep(5)
        Ccopy.view_close()

    is_feature_fulfilled()

    return


    for i, (name, ctrlset) in enumerate(reversed(control_sets)):
        # check if we need the goal or the last modified ctrlset
        if i == 0:
            action_next = goal_feature
        else:
            action_next = robust_plan[-1]

        print(ctrlset.getObjectives())
        # get all transient features of current ctrlset
        transient_curr = [obj.feat() for obj in ctrlset.getObjectives() if obj.get_OT() == ry.OT.sos]

        # get all immediate features of following ctrlset or goal
        immediate_next = [obj.feat() for obj in action_next.getObjectives()
                          if obj.get_OT() == ry.OT.eq or obj.get_OT() == ry.OT.ineq]

        # check which objectives are are not contained in transient feature
        implicit_features = [x for x in immediate_next if not any([is_equal_feature(x, y, C) for y in transient_curr])]

        # add implicit objectives to current controller as transient objectives
        for implicit_feature in implicit_features:

            ctrlset.addObjective(implicit_feature, ry.OT.eq, 0.005)

        robust_plan.append(ctrlset)

        print()
        print(f"Impliciticit features for {name} are:")
        for x in implicit_features: print(x.description(C))
        print(f"Transient features for {name} are:")
        for x in transient_curr: print(x.description(C))
        print(f"{immediate_next=}")
        print()


    #Start simulation of plan here
    # C.view()
    # tau = .01
    #
    # ctrl = ry.CtrlSolver(C, tau, 2)
    #
    # isDone = False
    #
    # for t in range(0, 10000):
    #
    #     ctrl = ry.CtrlSolver(C, tau, 2)
    #
    #     for i, c in enumerate(reversed(control_sets)):
    #
    #         if c[1].canBeInitiated(C):
    #             print(f"Initiating: {c[0]}")
    #             ctrl.set(c[1])
    #             break
    #         else:
    #             print(f"Cannot be initiated: {c[0]}")
    #             continue
    #
    #     if isDone:
    #
    #         #print(C.frame("bb1").info()["parent"])
    #         break
    #
    #     ctrl.update(C)
    #     q = ctrl.solve(C)
    #     C.setJointState(q)
    #     C.computeCollisions()
    #
    #     #     ctrl.report();
    #     #     C.watch(false, STRING(txt <<"t:" <<t));
    #     time.sleep(tau)
    #
    #
    # if isDone:
    #     print("Plan was finished")
    # else:
    #     print("time ran out")


if __name__ == '__main__':

    import sys
    print(sys.executable)

    build_tower()







