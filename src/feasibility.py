import libry as ry
import time as time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def check_switch_feasibility(C, controls, goal, vis=False, gnuplot=False, tolerance=0.1, verbose=False):

    # plan is feasible until proven otherwise
    plan_is_feasible = True

    # check feasibility with komo switches
    komo = ry.KOMO()
    komo.setModel(C, True)  # use swift collision engine
    komo.setTiming(len(controls), 1, 5., 1)

    # setup control cost
    komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    komo.addSquaredQuaternionNorms([], 3.)
    # we dont want collision
    komo.addObjective([], ry.FS.accumulatedCollisions, ["ALL"], ry.OT.eq, [1e1])

    # build a komo in which we only show controller siwtches
    for i, (name, controller) in enumerate(controls):

        # get the sos objectives of current controller
        for o in controller.getObjectives():
            if o.get_OT() == ry.OT.sos:
                f = o.feat()
                komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())

        for ctrlCommand in controller.getSymbolicCommands():
            if not ctrlCommand.isCondition():
                gripper, block = ctrlCommand.getFrameNames()
                if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    # TODO: find out when the object is released again, make a switch then, not hardcode
                    komo.addSwitch_stable(i, i + 2, "world", gripper, block)
                elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                    # TODO: find out when the object is released again, make a switch then, not hardcode
                    komo.addSwitch_stable(i, -1, gripper, "world", block)

    # solve or optime the given komo objectives
    komo.optimize()

    # get the report, which which generates the z.costReport file, which we can read
    komo.getReport(gnuplot)
    df_transient = pd.read_csv("z.costReport", index_col=None)
    df_transient.name = "transient features"

    # next, check for immediate constraints, and check if any are violated (by hand)
    controls_and_goal = list(controls)
    controls_and_goal.append(("goal", goal))

    C_copy = ry.Config()
    C_copy.copy(C)
    frames_state = 0  # needs to be initialized somehow
    frames_state = komo.getPathFrames(frames_state)

    # create a pandas data frame to store cost error for switches
    imm_objs = [o.feat().description(C_copy) for control in controls_and_goal for o in control[1].getObjectives()]
    df_immediate = pd.DataFrame(data=np.zeros((len(controls_and_goal), len(imm_objs))), columns=imm_objs)
    df_immediate.name = "immediate features"

    # check all immediate features
    for i, (name, control) in enumerate(controls_and_goal):
        # for the first switch, we can just use the initial frame state, therefore no need to set one from komo
        if i != 0:
            C_copy.setFrameState(frames_state[i-1])
        for o in control.getObjectives():
            if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                f = o.feat()
                error = f.eval(C_copy)[0]
                mse = np.sqrt(np.dot(error, error))
                description = f.description(C_copy)  # get the name
                df_immediate.loc[i, description] = mse  # set the mean squared error for switch

    for df in [df_transient, df_immediate]:
        # see which transient and immediate features are not fulfilled
        mse = df.to_numpy()
        features = df.columns
        for i_row, i_col in np.argwhere(mse > tolerance):
            plan_is_feasible = False
            if verbose:
                print(f'{df.name} {features[i_col]} at switch #{i_row} is not feasible!')

    # Visualize some results
    if vis:
        # create a figure with two plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 20))
        df_transient.plot(ax=ax1, title="Transient Features")
        df_immediate.plot(ax=ax2, title="Immediate Features")
        plt.show()

        # visualize the switches
        komo.view(False, "Feasibility Check")
        time.sleep(3)
        komo.view_play(.2, False)
        time.sleep(3)
        komo.view_close()

    return plan_is_feasible, komo
