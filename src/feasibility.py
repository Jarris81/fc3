import libry as ry
import time as time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from util.constants import type_block


def _get_ctrlset_description(C, ctrlset):
    for o in ctrlset.getObjectives():
        f = o.feat()
        print(f"{f.getFS()}, {f.getFrameNames(C)}, {o.get_OT()}, {f.getScale()}")


def check_switch_chain_feasibility(C, controls, goal, scene_objects, tolerance=0.1, verbose=False):

    # plan is feasible until proven otherwise
    plan_is_feasible = True

    C_temp = ry.Config()
    C_temp.copy(C)

    # TODO: get gripper and block names as input
    gripper = "R_gripper"

    holding_list = {}
    holding = {}
    for block_name in scene_objects[type_block]:
        block = C_temp.frame(block_name)
        if "parent" in block.info() and block.info()["parent"] == gripper:
            holding_list[block_name] = [True]
            holding[block_name] = True
        else:
            holding_list[block_name] = [False]
            holding[block_name] = False

    komo = ry.KOMO()
    komo.setModel(C_temp, False)  # use swift collision engine
    komo.setTiming(len(controls), 1, 5., 1)

    komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    komo.addSquaredQuaternionNorms([], 3.)

    # build a komo in which we only show controller switches
    for i, (name, controller) in enumerate(controls):

        # get the sos objectives of current controller
        for o in controller.getObjectives():
            f = o.feat()
            # we dont care about control objectives
            if o.get_OT() == ry.OT.sos and "qItself" not in f.description(C_temp):
                komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C_temp), o.get_OT(), f.getScale(),
                                  o.getOriginalTarget())

        for ctrlCommand in controller.getSymbolicCommands():
            gripper, block = ctrlCommand.getFrameNames()
            if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                holding[block] = True
            elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                holding[block] = False

        # append blocks
        for block in holding:
            holding_list[block].append(holding[block])

    # go over every switch for each block
    for block in holding_list:
        hold_list = holding_list[block]
        holding_duration = [[hold_list[0], 0, 0]]
        last_hold = hold_list[0]
        for i, hold in enumerate(hold_list):
            # specifying the first grab/release is redundant, since no switch
            if i == 0:
                continue
            else:
                # if are same, add one to duration
                if last_hold is hold:
                    holding_duration[-1][-1] += 1
                else:
                    # switch was made!
                    holding_duration.append([hold, holding_duration[-1][-1], 1 + holding_duration[-1][-1]])
            last_hold = hold
        for j, (grab, start, end) in enumerate(holding_duration):
            if j == 0:
                continue
            if grab:
                # grab
                if end == len(controls):
                    komo.addSwitch_stable(start, -1, "world", gripper, block)
                else:
                    komo.addSwitch_stable(start, end, "world", gripper, block)
            else:
                # open
                if end == len(controls):
                    komo.addSwitch_stable(start, -1, gripper, "world", block)
                else:
                    komo.addSwitch_stable(start, end, gripper, "world", block)

    # solve or optime the given komo objectives
    komo.optimize()

    # get the report, which which generates the z.costReport file, which we can read
    komo.getReport(verbose)
    df_transient = pd.read_csv("z.costReport", index_col=None)
    df_transient.name = "Transient features:"

    # next, check for immediate constraints, and check if any are violated
    controls_and_goal = list(controls)
    controls_and_goal.append(("goal", goal))

    C_temp = ry.Config()
    C_temp.copy(C)
    frames_state = 0  # needs to be initialized somehow
    frames_state = komo.getPathFrames(frames_state)

    # create a pandas data frame to store cost error for switches with unique features
    imm_objs = set([o.feat().description(C_temp) for control in controls_and_goal for o in control[1].getObjectives()])
    df_immediate = pd.DataFrame(data=np.zeros((len(controls_and_goal), len(imm_objs))), columns=imm_objs)
    df_immediate.name = "immediate features"

    # check all immediate features
    for i, (name, control) in enumerate(controls_and_goal):
        # for the first switch, we can just use the initial frame state, therefore no need to set one from komo
        if i != 0:
            C_temp.setFrameState(frames_state[i-1])
        for o in control.getObjectives():
            if o.get_OT() == ry.OT.eq or o.get_OT() == ry.OT.ineq:
                f = o.feat()
                # TODO: check if this the right way for an error
                error = f.eval(C_temp)[0]
                mse = np.sqrt(np.dot(error, error))
                if f.getFS() == ry.FS.pairCollision_negScalar:
                    mse = error
                description = f.description(C_temp)  # get the name
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
    if verbose:
        # create a figure with two plot
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
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
