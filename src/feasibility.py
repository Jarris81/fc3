import libry as ry
import time as time


def check_feasibility(C, controls, steps_per_keyframe=1, vis=False, goal=None):
    # check feasibility with komo switches
    komo = ry.KOMO()
    komo.setModel(C, True)  # use swift collision engine

    if steps_per_keyframe == 1:
        komo.setTiming(len(controls), steps_per_keyframe, 5., 1)
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        komo.addSquaredQuaternionNorms([], 3.)
    else:
        komo.setTiming(len(controls), steps_per_keyframe, 5., 2)
        komo.add_qControlObjective([], 2)
        komo.addSquaredQuaternionNorms([], 3.)

    # we dont want collision
    komo.addObjective([], ry.FS.accumulatedCollisions, ["ALL"], ry.OT.eq, [1e1])

    #  new komo: get transient objectives from current, and immediate from next, and see if they are feasible
    for i in range(len(controls)):

        # get the sos objectives of current controller
        for o in controls[i][1].getObjectives():
            if o.get_OT() == ry.OT.sos:
                f = o.feat()
                #if steps_per_keyframe == 1:
                komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
                #komo.addObjective([i + 1], ry.FS.accumulatedCollisions, ["ALL"], ry.OT.ineq, [1e2])
                # else:
                #     komo.addObjective([i + 1, i + 2], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())

        # check of we are not in the last switch, which is the switch from end to goal
        if i != len(controls) - 1:
            # get the immediate objectives of following controller
            for o in controls[i+1][1].getObjectives():
                if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                    f = o.feat()
                    #if steps_per_keyframe == 1:
                    #komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
                    # else:
                    #     komo.addObjective([i + 1, i + 2], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(),
                    #                       f.getTarget())
            for ctrlCommand in controls[i + 1][1].getSymbolicCommands():
                if not ctrlCommand.isCondition():
                    gripper, block = ctrlCommand.getFrameNames()
                    if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                        # TODO: find out when the object is released again, make a switch then, not hardcode
                        komo.addSwitch_stable(i + 1, i + 3, "world", gripper, block)
                    elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                        # TODO: find out when the object is released again, make a switch then, not hardcode
                        komo.addSwitch_stable(i + 1, -1, gripper, "world", block)

        else:
            for o in goal.getObjectives():
                if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                    f = o.feat()
                    komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())

    komo.optimize()

    if vis:
        komo.view(False, "Feasibility Check")

        time.sleep(2.5)

        komo.view_play(.2, False)

        time.sleep(5)
        report = komo.getReport()
        time.sleep(30)

    print(komo.getConstraintViolations())

    return komo

