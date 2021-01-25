import time
import libry as ry

import controllers as con
import util.domain_tower as dt
from testing.tower_planner import get_plan, get_goal_feature
from util.setup_env import setup_tower_env



"""
Build a tower with the provided plan
"""


def setup_scene():

    R, C = setup_tower_env(2)
    C.view()
    #time.sleep(10)

    return R, C


if __name__ == '__main__':

    # get all actions needed to build a tower
    actions = [
        con.ApproachBlock(),
        con.PlaceOn(),
        con.CloseGripper(),
        con.OpenGripper()
    ]

    name2con = {x.name: x for x in actions}

    _, C,  block_names = setup_tower_env(3)

    gripper_name = "R_gripper"

    #block_names = tuple(block_names)

    all_objects = {
        dt.type_block: block_names,  # , "b3"),
        dt.type_gripper: (gripper_name,)
    }
    # get plan
    plan, goal = get_plan(False, actions, all_objects)

    if plan:
        print("Found Plan:")
        for action in plan:
            print(action)


    # convert simple actions to CtrlSet

    control_sets = []

    for grounded_action in plan:
        obj_frames = grounded_action.sig[1:]
        controller = name2con[grounded_action.sig[0]]  # the actual control
        control_sets.append((grounded_action, controller.get_grounded_control_set(C, obj_frames)))
        #control_sets.append(controller.get_grounded_control_set(C, obj_frames))

    switches_count=0

    for i, (name, control) in enumerate(control_sets):
        # convert to KOMO, and add to state

        # check if there are any symbolic commands which have switches
        if len(control.getSymbolicCommands()):
            for ctrlCommand in control.getSymbolicCommands():
                if ctrlCommand.isCondition() and ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    switches_count += 1
                elif ctrlCommand.isCondition() and ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                    switches_count += 1


    print()
    print(f"switches_count is {switches_count}")
    print()

    # check feasibility with komo switches
    komo = ry.KOMO()

    number_controls = len(control_sets)

    steps_per_keyframe = 10
    komo.setModel(C, False)

    if steps_per_keyframe == 1:
        komo.setTiming(switches_count+1, steps_per_keyframe, 5., 1)
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        komo.addSquaredQuaternionNorms([], 3.)
    else:
        komo.setTiming(switches_count+1, steps_per_keyframe, 5., 2)
        komo.add_qControlObjective([], 2)
        komo.addSquaredQuaternionNorms([], 3.)

    i = 0
    for (name, control) in control_sets:
        print(name)
        for o in control.getObjectives():
            f = o.feat()
            komo.addObjective([i+1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
        for ctrlCommand in control.getSymbolicCommands():
            if not ctrlCommand.isCondition():

                if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    # add switch
                    f1, f2 = ctrlCommand.getFrameNames()
                    komo.addSwitch_stable(i+1, i+2, "world", f1, f2)
                    # hack, because block jumps back to original position
                    if f2 == "b1":
                         komo.addSwitch_stable(i+1, i+2, "world", "world", "b2")
                elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                    # add switch
                    f1, f2 = ctrlCommand.getFrameNames()

                    if f2 == "b2":
                        komo.addSwitch_stable(i+1, i+2, f1, "world", f2)
                    if f2 == "b1":
                        komo.addSwitch_stable(i+1, i+2, f1, "world", f2)
                        komo.addSwitch_stable(i + 1, i + 2, "world", "world", "b2")

                # make sure we go to next switch
                i = i + 1
    # hack, add switches at end
    komo.addSwitch_stable(i + 1, -1, "world", "world", "b1")
    komo.addSwitch_stable(i + 1, -1, "world", "world", "b2")

    komo.optimize()

    komo.view(False, "result")

    time.sleep(10)

    # %%

    komo.view_play(.2, False)

    time.sleep(10)



    # get goal condition
    goal_feature = get_goal_feature(C, goal)

    # get implicit conditions
    robust_plan = []

    # for i, action in enumerate(reversed(control_sets)):
    #     print(i)
    #     if i == 0:
    #         action_next = goal_feature
    #     else:
    #         action_next = robust_plan[-1]
    #     # get all inital conditions from following action
    #     print(action_next.getObjectives())
    #     for o in action_next.getObjectives():
    #         print(o.get_OT())
    #     end_cond = [x.feat() for x in action.getObjectives() if x.get_OT() == ry.OT.sos]
    #     print(end_cond)
    #     x = [l.feat() for l in action_next.getObjectives()
    #          if l.get_OT() == ry.OT.eq or l.get_OT() == ry.OT.ineq and l.feat() not in end_cond]
    #     print(x)
    #
    #     for a in x:
    #         for b in end_cond:
    #             print(b.description(C))
    #             print(a.description(C))
    #             print(dir(a))
    #
    #
    # print(dir(goal_feature.getObjectives()[0]))







    # Start simulation of plan here
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





