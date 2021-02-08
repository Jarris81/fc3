import libry as ry
import time as time


def check_feasibility(C, controls, steps_per_keyframe=1, hack=True, vis=False):

    # figure out the amount of switches for feasibility check
    switches_count = 0
    for name, control in controls:
        for symCtrl in control.getSymbolicCommands():
            if (symCtrl.getCommand() == ry.SC.CLOSE_GRIPPER or symCtrl.getCommand() == ry.SC.OPEN_GRIPPER) \
                    and symCtrl.isCondition():
                switches_count += 1
                break

    # check feasibility with komo switches
    komo = ry.KOMO()
    komo.setModel(C, False)

    if steps_per_keyframe == 1:
        komo.setTiming(switches_count, steps_per_keyframe, 5., 1)
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        komo.addSquaredQuaternionNorms([], 3.)
    else:
        komo.setTiming(switches_count, steps_per_keyframe, 5., 2)
        komo.add_qControlObjective([], 2)
        komo.addSquaredQuaternionNorms([], 3.)

    # build komo
    i = 0
    for (name, control) in controls:
        for o in control.getObjectives():
            f = o.feat()
            komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
        for ctrlCommand in control.getSymbolicCommands():
            if not ctrlCommand.isCondition():
                gripper, block = ctrlCommand.getFrameNames()
                if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    # add switch
                    komo.addSwitch_stable(i + 1, i + 2, "world", gripper, block)
                elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                    # add switch
                    komo.addSwitch_stable(i + 1, -1, gripper, "world", block)
                # make sure we go to next switch
                i = i + 1

    komo.optimize()

    if vis:
        komo.view(False, "result")

        time.sleep(5)

        komo.view_play(.2, False)

        time.sleep(5)
    report = komo.getReport()

    time.sleep(10)
    return komo


def check_feasibility2(C, controls, steps_per_keyframe=1, hack=True, vis=False, goal=None):
    # check feasibility with komo switches
    komo = ry.KOMO()
    komo.setModel(C, False)

    if steps_per_keyframe == 1:
        komo.setTiming(len(controls), steps_per_keyframe, 5., 1)
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        #komo.addSquaredQuaternionNorms([], 3.)
    else:
        komo.setTiming(len(controls), steps_per_keyframe, 5., 2)
        komo.add_qControlObjective([], 2)
        komo.addSquaredQuaternionNorms([], 3.)

    #  new komo: get transient objectives from current, and immediate from next, and see if they are feasible

    for i in range(len(controls)):

        # get the sos objectives of current controller
        for o in controls[i][1].getObjectives():
            if o.get_OT() == ry.OT.sos:
                f = o.feat()
                komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())

        if i != len(controls) - 1:
            # get the immediate objectives of following controller
            for o in controls[i+1][1].getObjectives():
                if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                    f = o.feat()
                    komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
            for ctrlCommand in controls[i + 1][1].getSymbolicCommands():
                if not ctrlCommand.isCondition():
                    gripper, block = ctrlCommand.getFrameNames()
                    if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                        # add switch
                        komo.addSwitch_stable(i + 1, i + 3, "world", gripper, block)
                        # komo.add_qControlObjective([i+1], 1, 1e1)
                        # for o in controls[i-1][1].getObjectives():
                        #     f = o.feat()
                        #     komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(),
                        #                       f.getTarget())
                    elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                        # add switch
                        komo.addSwitch_stable(i + 1, -1, gripper, "world", block)
                        # komo.add_qControlObjective([i + 1], 1, 1e1)
                        # for o in controls[i-1][1].getObjectives():
                        #     f = o.feat()
                        #     komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(),
                        #                       f.getTarget())
        else:
            for o in goal.getObjectives():
                if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                    f = o.feat()
                    komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())


        # add switches




    # build komo
    # for i, (name, control) in enumerate(controls):
    #     print(i, name)
    #     for o in control.getObjectives():
    #         f = o.feat()
    #         komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())
    #
    #     for ctrlCommand in control.getSymbolicCommands():
    #         if not ctrlCommand.isCondition():
    #             gripper, block = ctrlCommand.getFrameNames()
    #             if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
    #                 # add switch
    #                 komo.addSwitch_stable(i + 1, i + 3, "world", gripper, block)
    #                 #komo.add_qControlObjective([i+1], 1, 1e1)
    #                 # for o in controls[i-1][1].getObjectives():
    #                 #     f = o.feat()
    #                 #     komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(),
    #                 #                       f.getTarget())
    #             elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
    #                 # add switch
    #                 komo.addSwitch_stable(i + 1, -1, gripper, "world", block)
    #                 #komo.add_qControlObjective([i + 1], 1, 1e1)
    #                 # for o in controls[i-1][1].getObjectives():
    #                 #     f = o.feat()
    #                 #     komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(),
    #                 #                       f.getTarget())

    komo.optimize()

    if vis:
        komo.view(False, "result")

        time.sleep(2.5)

        komo.view_play(.2, False)

        time.sleep(5)
    report = komo.getReport()
    time.sleep(30)
    return komo

