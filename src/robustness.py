import libry as ry
import time
import numpy as np


def is_equal_feature(f1, f2, C):
    """
    Checks if two features are equal.
    Not checking objective type!
    """
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
    """
    Check if two symbolic commands are the same.
    We are not checking for the condition!
    """

    if not sc1.getFrameNames() == sc2.getFrameNames():
        return False
    if not sc1.getCommand() == sc2.getCommand():
        return False
    return True


def get_implicit_objectives_chain(C, komo_feasy, current, follow, step, vis=False, verbose=False):
    """

    """
    # make a copy of the current Config
    C_temp = ry.Config()
    C_temp.copy(C)

    # store implicit features
    implicit_objectives_list = []
    # additionally we need to get implicit symbolic commands (pretty much exactly like RLDS paper)
    implicit_sym_commands_list = []

    # i
    if step <= komo_feasy.getT():
        frames_state = 0  # needs to be initialized somehow
        frames_state = komo_feasy.getPathFrames(frames_state)
        C_temp.setFrameState(frames_state[-step])

    # check all objectives of following controller, see if they need to be add to current as implicit
    for follow_obj in follow.getObjectives():
        # only care about immediate objectives
        if follow_obj.get_OT() == ry.OT.eq or follow_obj.get_OT() == ry.OT.ineq:

            # get the feature, and see if it is fulfilled in switch
            follow_feat = follow_obj.feat()
            error = follow_feat.eval(C_temp)[0]  # evaluate feature in frame switch

            # if all errors are smaller t
            if np.all(np.sqrt(error * error) < 1e-1):
                # check if feature is not already in current controller
                is_implicit = True
                for current_obj in current.getObjectives():
                    if is_equal_feature(current_obj.feat(), follow_feat, C_temp):
                        is_implicit = False
                        break  # only need to find one objective which is equal
                if is_implicit:
                    implicit_objectives_list.append(follow_feat)

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

    if vis:
        C_temp.view()
        time.sleep(1)
        C_temp.view_close()

    return implicit_objectives_list, implicit_sym_commands_list


def get_robust_system(C, komo_feasy, controllers, goal_controller, verbose=False):

    robust_controller_chain = []

    # start from last controller (closest to goal), work up
    for i, (name, ctrlset) in enumerate(reversed(controllers)):
        # check if we need the goal or the last modified ctrlset
        if i == 0:
            action_next = goal_controller
        else:
            action_next = robust_controller_chain[-1][1]
        if verbose:
            print("-"*20)
            print(f"Implicit Features for {name}:")

        step = i + 2  # need a step of 2 more, because start from back (0 becomes -1) and dont need to see goal switch
        implicit_features, implicit_scs = get_implicit_objectives_chain(C, komo_feasy, ctrlset, action_next, step, verbose)

        # add implicit objectives to current controller as transient objectives
        for implicit_feature in implicit_features:
            if implicit_feature.getFS() == ry.FS.pairCollision_negScalar:
                ctrlset.addObjective(implicit_feature, ry.OT.ineq, -1)  # TODO: need to get the same OT, could also be ineq
            else:
                ctrlset.addObjective(implicit_feature, ry.OT.eq, -1)
            if verbose:
                print(implicit_feature.description(C), )
        for implicit_sc in implicit_scs:
            ctrlset.addSymbolicCommand(implicit_sc.getCommand(), implicit_sc.getFrameNames(), True)  # always condition
            if verbose:
                print(implicit_sc.getCommand()) # TODO: add description

        robust_controller_chain.append((name, ctrlset))

    return robust_controller_chain

def get_robust_system_2(C, controllers, goal_controller, verbose=True):
    """
    Only use the goal state(which we need to describe somehow) to get implicit conditions
    @param C:
    @param controllers:
    @param goal_controller:
    @param verbose:
    @return:
    """

    robust_controller_chain = []

    C_temp = ry.Config()
    C_temp.copy(C)

    for i, (name, ctrlset) in enumerate(reversed(controllers)):

        if i == 0:
            action_next = goal_controller
        else:
            action_next = robust_controller_chain[-1][1]

        step = i + 2  # need a step of 2 more, because start from back (0 becomes -1) and dont need to see goal switch

        # do a 1 step komo from current controller, and see which objectives are missing
        komo = ry.KOMO()
        komo.setModel(C, True)  # use swift collision engine
        komo.setTiming(1, 1, 5., 1)

        # setup control cost
        komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
        komo.addSquaredQuaternionNorms([], 3.)

        for o in ctrlset.getObjectives():
            #if o.get_OT() == ry.OT.sos:
            f = o.feat()
            komo.addObjective([1], f.getFS(), f.getFrameNames(C), o.get_OT(), f.getScale(), f.getTarget())

        for ctrlCommand in ctrlset.getSymbolicCommands():
            if ctrlCommand.isCondition():
                gripper, block = ctrlCommand.getFrameNames()
                if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                    komo.addSwitch_stable(0, -1, "world", gripper, block)


        komo.optimize()

        frames_state = 0  # needs to be initialized somehow
        frames_state = komo.getPathFrames(frames_state)
        C_temp.setFrameState(frames_state[-1])

        implicit_features = []

        # implicit features are all those features of following, which are not fullfilled in komo solution
        for follow_obj in action_next.getObjectives():
            # only care about immediate objectives
            if follow_obj.get_OT() == ry.OT.eq or follow_obj.get_OT() == ry.OT.ineq:

                # get the feature, and see if it is fulfilled in switch
                follow_feat = follow_obj.feat()
                error = follow_feat.eval(C_temp)[0]  # evaluate feature in frame switch

                # if all errors are smaller t
                if np.any(np.sqrt(error * error) > 1e-1):
                    # check if feature is not already in current controller
                    implicit_features.append(follow_feat)

        implicit_scs = []

        for sym_obj in action_next.getSymbolicCommands():
            if sym_obj.isCondition():
                is_implicit = True
                # check if the current controller has that same symbolic command as run
                for current_sc in ctrlset.getSymbolicCommands():
                    if is_equal_sym_command(sym_obj, current_sc):
                        is_implicit = False
                        break
                if is_implicit:
                    implicit_scs.append(sym_obj)

        if verbose:
            print("-"*20)
            print(f"Implicit Features for {name}:")
        # add implicit objectives to current controller as transient objectives
        for implicit_feature in implicit_features:
            if implicit_feature.getFS() == ry.FS.pairCollision_negScalar:
                ctrlset.addObjective(implicit_feature, ry.OT.ineq,
                                     -1)  # TODO: need to get the same OT, could also be ineq
            else:
                ctrlset.addObjective(implicit_feature, ry.OT.eq, -1)
            if verbose:
                print(implicit_feature.description(C), )
        for implicit_sc in implicit_scs:
            ctrlset.addSymbolicCommand(implicit_sc.getCommand(), implicit_sc.getFrameNames(), True)  # always condition
            if verbose:
                print(implicit_sc.getCommand())  # TODO: add description

        if verbose:
            komo.view(False, f"Implicit conditions check for {name}")
            time.sleep(2)
            komo.view_close()


        robust_controller_chain.append((name, ctrlset))

    return robust_controller_chain
