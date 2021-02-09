import time
import libry as ry
import numpy as np

import controllers as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_controller
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


    # get goal condition
    goal_controller = get_goal_controller(C, goal)
    # get implicit conditions
    robust_plan = []

    # check if plan is feasible in current config
    komo_feasy = check_feasibility2(C, control_sets, steps_per_keyframe=1, hack=False, vis=False, goal=goal_controller)


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

    def is_equal_sym_command(sc1, sc2):

        if not sc1.getFrameNames() == sc2.getFrameNames():
            return False
        if not sc1.getCommand() == sc2.getCommand():
            return False

        return True
    # Better: use the feasibility komo, to see if the immediate conditions of the following controller are present
    # if yes, they are implied feature
    def get_implied_features(step, current, follow, vis=False):

        Ccopy = ry.Config()
        Ccopy.copy(C)

        frames_state = 0  # needs to be initialized somehow
        frames_state = komo_feasy.getPathFrames(frames_state)

        implicit_features_list = []

        if step < 9:
            Ccopy.setFrameState(frames_state[-step])

        for follow_obj in follow.getObjectives():
            if follow_obj.get_OT() == ry.OT.eq or follow_obj.get_OT() == ry.OT.ineq:
                follow_feat = follow_obj.feat()
                result = follow_feat.eval(Ccopy)  # evaluate feature in frame switch
                y = result[0]  # get the error
                #print(follow_feat.description(Ccopy))
                #print(f"Error: {np.sqrt(y*y)}")

                if np.all(np.sqrt(y*y) < 1e-2):
                    # check if feature is not already in current controller
                    is_implicit = True
                    for current_obj in current.getObjectives():
                        if is_equal_feature(current_obj.feat(), follow_feat, Ccopy):
                            is_implicit = False
                            break

                    if is_implicit:
                        implicit_features_list.append(follow_feat)
                    # print(f"Is implicit: {is_implicit}")

        # additionally we need to get implicit symbolic commands (pretty much exactly like RLDS paper)
        implicit_sym_commands_list = []

        for sym_obj in follow.getSymbolicCommands():
            if sym_obj.isCondition():
                is_implicit = True
                # check if the current controller has that same symbolic command as run
                for current_sc in current.getSymbolicCommands():
                    if is_equal_sym_command(sym_obj, current_sc):
                        is_implicit = False
                        break
                if is_implicit:
                    implicit_sym_commands_list.append(sym_obj)
                    print(f"Is implicit SC: {is_implicit}")


        for x in implicit_features_list: print(x.description(C))
        if vis:
            Ccopy.view()
            time.sleep(1)
            Ccopy.view_close()

        return implicit_features_list, implicit_sym_commands_list

    for i, (name, ctrlset) in enumerate(reversed(control_sets)):
        # check if we need the goal or the last modified ctrlset
        if i == 0:
            action_next = goal_controller
        else:
            action_next = robust_plan[-1][1]

        print(f"Implicit Features for {name}:")
        implicit_features, implicit_scs = get_implied_features(i + 2, ctrlset, action_next, vis=False)


        # add implicit objectives to current controller as transient objectives
        for implicit_feature in implicit_features:

            # problem: implicit features come from sos, therefore they might have nct been solved perfectly?
            ctrlset.addObjective(implicit_feature, ry.OT.eq, -1) # need to get the same OT here, could also be ineq

        for implicit_sc in implicit_scs:
            ctrlset.addSymbolicCommand(implicit_sc.getCommand(), implicit_sc.getFrameNames(), True)  # always condition

        if "OPEN" in name.sig[0]:
            ctrlset.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", "b2"), False)
        robust_plan.append((name, ctrlset))

    # return

    # Start simulation of plan here
    C.view()
    tau = .01

    isDone = False

    for t in range(0, 10000):

        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, (name, c) in enumerate(robust_plan):

            if c.canBeInitiated(C):
                print(f"Initiating: {name}")
                ctrl.set(c)
                if 4 == i:

                    print(f"{name} b1 working")
                    c = ry.CtrlSet()
                    c.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", "b2"), False)  # isCondition=True, therefore respected in initial feasibility
                    #c.addObjective(C.feature(ry.FS.scalarProductZZ, ["b2", "b1"], [1e1], [1]), ry.OT.eq, -1)
                    #c.addObjective(C.feature(ry.FS.positionRel, ["b2", "b1"], [1e1], [0, 0, -0.1]), ry.OT.eq,
                                              # -1)
                    ctrl.set(c)
                break
            else:
                print(f"Cannot be initiated: {name}, {i},")
                continue

        if isDone:


            break

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()

        if "parent" in C.frame("b2").info():
            if "world" == C.frame("b2").info()["parent"]:
                print("b2 is in world")
            elif "R_gripper" == C.frame("b2").info()["parent"]:
                print("b2 is attached")

        #     ctrl.report();
        #     C.watch(false, STRING(txt <<"t:" <<t));
        time.sleep(tau)


    if isDone:
        print("Plan was finished")
    else:
        print("time ran out")


if __name__ == '__main__':

    import sys
    print(sys.executable)

    build_tower()







