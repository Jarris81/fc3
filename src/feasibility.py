import libry as ry
import time as time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def _get_ctrlset_description(C, ctrlset):
    for o in ctrlset.getObjectives():
        f = o.feat()
        print(f"{f.getFS()}, {f.getFrameNames(C)}, {o.get_OT()}, {f.getScale()}")


def check_switch_chain_feasibility(C, controls, goal, tolerance=0.1, verbose=False):

    # plan is feasible until proven otherwise
    plan_is_feasible = True

    C_copy = ry.Config()
    C_copy.copy(C)

    if verbose:
        C_copy.view()
        time.sleep(3)
        C_copy.view_close()

    # check feasibility with komo switches


    # setup control cost
    #komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    #komo.addSquaredQuaternionNorms([], 3.)
    # we dont want collision
    #komo.addObjective([], ry.FS.accumulatedCollisions, ["ALL"], ry.OT.eq, [1e1])

    # init the grippers, and check when they are carrying an object
    # get all Grippers in the scene
    num_grippers = 2  # TODO get gripper names from C and see who holds initially
    gripper_hold = {"R_gripper": (0, None), "L_gripper": (0, None)}


    holding_list = {}
    holding = {}
    for block_name in ["b1", "b2", "b3"]:
        block = C_copy.frame(block_name)
        print(block.info())
        if "parent" in block.info() and block.info()["parent"] == "R_gripper":
            # quick hack: take block out of frame, and add grab control
            # C.attach("world", block_name)
            # gripper = "R_gripper"
            # gripper_center = gripper + "Center"
            #
            # #  block needs to be close to block
            # grab = ry.CtrlSet()
            # grab.addObjective(
            #     C.feature(ry.FS.distance, [block_name, gripper_center], [1e1]),
            #     ry.OT.eq, -1)
            # # condition, nothing is in hand of gripper
            # grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block_name), True)
            # grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block_name), False)

            #controls.insert(0, ("grab1",grab))
            #controls = controls[2:]
            holding_list[block_name] = [True]
            holding[block_name] = True
        else:
            holding_list[block_name] = [False]
            holding[block_name] = False

    print([c[0] for c in controls])

    komo = ry.KOMO()
    komo.setModel(C_copy, False)  # use swift collision engine
    komo.setTiming(len(controls), 1, 5., 1)

    # komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    # komo.addSquaredQuaternionNorms([], 3.)

    # build a komo in which we only show controller switches
    for i, (name, controller) in enumerate(controls):

        # # get the sos objectives of current controller
        # for o in controller.getObjectives():
        #     f = o.feat()
        #     des = f.description(C_copy)
        #     # if "qItself" in des:
        #     #     continue
        #     if o.get_OT() == ry.OT.sos:
        #         komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C_copy), o.get_OT(), f.getScale(), f.getTarget())

        for ctrlCommand in controller.getSymbolicCommands():
            gripper, block = ctrlCommand.getFrameNames()
            if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                holding[block] = True
                    #komo.addSwitch_stable(i, i + 2, "world", gripper, block)
            elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                holding[block] = False

        #append blocks
        for block in holding:
            holding_list[block].append(holding[block])

    # go over every switch for each block

    for block in holding_list:
        hold_list = holding_list[block]
        holding_duration = [[hold_list[0], 0, 0]]
        last_hold = hold_list[0]
        for i, hold in enumerate(hold_list):
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
            gripper = "R_gripper"
            if j == 0:
                continue
            if grab:
                # grab
                if end == len(controls):
                    komo.addSwitch_stable(start, -1, "world", gripper, block)
                else:
                    print(f"{end=}")
                    komo.addSwitch_stable(start, end, "world", gripper, block)
            elif not grab:
                # open
                if end == len(controls):
                    komo.addSwitch_stable(start, -1, gripper, "world", block)
                else:
                    print(start)
                    komo.addSwitch_stable(start, end, gripper, "world", block)

    # build a komo in which we only show controller switches
    for i, (name, controller) in enumerate(controls):

        # get the sos objectives of current controller
        for o in controller.getObjectives():
            f = o.feat()
            if o.get_OT() == ry.OT.sos:
                print(f.getFS())
                print(f.getScale())
                print(o.getOriginalTarget())
                komo.addObjective([i + 1], f.getFS(), f.getFrameNames(C_copy), o.get_OT(), f.getScale(),
                                  o.getOriginalTarget())

    # solve or optime the given komo objectives
    komo.optimize()

    # get the report, which which generates the z.costReport file, which we can read
    komo.getReport(verbose)
    df_transient = pd.read_csv("z.costReport", index_col=None)
    df_transient.name = "Transient features:"

    # next, check for immediate constraints, and check if any are violated
    controls_and_goal = list(controls)
    controls_and_goal.append(("goal", goal))

    C_copy = ry.Config()
    C_copy.copy(C)
    frames_state = 0  # needs to be initialized somehow
    frames_state = komo.getPathFrames(frames_state)

    # create a pandas data frame to store cost error for switches with unique features
    imm_objs = set([o.feat().description(C_copy) for control in controls_and_goal for o in control[1].getObjectives()])
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
                if f.getFS() == ry.FS.pairCollision_negScalar:
                    mse = error
                description = f.description(C_copy)  # get the name
                df_immediate.loc[i, description] = mse  # set the mean squared error for switch

    for df in [df_transient, df_immediate]:
        # see which transient and immediate features are not fulfilled
        mse = df.to_numpy()
        features = df.columns
        for i_row, i_col in np.argwhere(mse > tolerance):
            plan_is_feasible = False
            if verbose:
                #print(mse[i_row, i_col])
                print(f'{df.name} {features[i_col]} at switch #{i_row} is not feasible!')

    # Visualize some results
    if verbose:
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
