import libry as ry
import time as time


def check_feasibility(C, controls, steps_per_keyframe=20):

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
        komo.setTiming(switches_count + 1, steps_per_keyframe, 5., 1)
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        komo.addSquaredQuaternionNorms([], 3.)
    else:
        komo.setTiming(switches_count + 1, steps_per_keyframe, 5., 2)
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
                if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    # add switch
                    f1, f2 = ctrlCommand.getFrameNames()
                    komo.addSwitch_stable(i + 1, i + 2, "world", f1, f2)
                    # TODO: remove this, by fixing addSwitch_stable
                    # hack, because block jumps back to original position
                    if f2 == "b1":
                        komo.addSwitch_stable(i + 1, i + 2, "world", "world", "b2")
                elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                    # add switch
                    f1, f2 = ctrlCommand.getFrameNames()

                    # TODO: remove this, by fixing addSwitch_stable
                    if f2 == "b2":
                        komo.addSwitch_stable(i + 1, i + 2, f1, "world", f2)
                    if f2 == "b1":
                        komo.addSwitch_stable(i + 1, i + 2, f1, "world", f2)
                        komo.addSwitch_stable(i + 1, i + 2, "world", "world", "b2")

                # make sure we go to next switch
                i = i + 1
    # TODO: remove this, by fixing addSwitch_stable
    komo.addSwitch_stable(i + 1, -1, "world", "world", "b1")
    komo.addSwitch_stable(i + 1, -1, "world", "world", "b2")

    komo.optimize()

    komo.view(False, "result")

    time.sleep(5)

    komo.view_play(.2, False)

    report = komo.getReport()
    time.sleep(10)
