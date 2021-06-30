import libry as ry
import time
import numpy as np
import networkx as nx

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


def get_robust_system2(C, komo_feasy, controllers, goal_controller, verbose=False):

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


def get_robust_chain(C, controllers, goal_controller, verbose=False):
    """
    We want to get the implicit features without states (configs) and only use the goal geometric description
    @param C: cuurent config of world
    @param controllers: controller chain
    @param goal_controller: goal controller
    @param verbose: True if you want some info
    @return:
    """

    robust_controller_chain = []

    C_temp = ry.Config()
    C_temp.copy(C)

    for i, (edge, name, ctrlset) in enumerate(reversed(controllers)):

        if i == 0:
            action_next = goal_controller
        else:
            action_next = robust_controller_chain[-1][-1]

        # do a 1-step komo from current controller, and see which objectives are missing
        komo = ry.KOMO()
        komo.setModel(C, True)  # use swift collision engine
        komo.setTiming(1, 1, 5., 1)

        # setup control cost
        komo.add_qControlObjective([], 1, 1e-1)
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

        # solve komo
        komo.optimize()

        # get the config with all features applied from current controller
        frames_state = 0  # needs to be initialized somehow
        frames_state = komo.getPathFrames()
        C_temp.setFrameState(frames_state[-1])

        implicit_features_tuples = []

        # implicit features are all those features of next/follow controller, which are NOT fulfilled in komo solution
        for follow_obj in action_next.getObjectives():
            # only care about immediate objectives
            obj_type = follow_obj.get_OT()
            if obj_type == ry.OT.eq or obj_type == ry.OT.ineq:

                # get the feature, and see if it is fulfilled in switch
                follow_feat = follow_obj.feat()
                error = follow_feat.eval(C_temp)[0]  # evaluate feature in frame switch

                # if all errors are smaller than tolerance, feature is NOT fulfilled
                if np.any(np.sqrt(error * error) > 1e-1):
                    # check if feature is not already in current controller
                    implicit_features_tuples.append((follow_feat, obj_type))

        # symbolic commands are handled exactly like in RLDS paper
        # if current controller does not have that condition as run command, it is implicit
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
        for implicit_feature, obj_type in implicit_features_tuples:
            ctrlset.addObjective(implicit_feature, ry.OT.eq, -1)  # TODO: need to get the same OT, could also be ineq
            if verbose:
                print(implicit_feature.description(C), )
        for implicit_sc in implicit_scs:
            ctrlset.addSymbolicCommand(implicit_sc.getCommand(), implicit_sc.getFrameNames(), True)  # always condition
            if verbose:
                print(implicit_sc.getCommand())  # TODO: add description

        if verbose:
            komo.view(True, f"Implicit conditions check for {name}")
            #time.sleep(5)
            komo.view_close()

        robust_controller_chain.append((edge, name, ctrlset))

    # return in reversed order, so consistent with way put in
    return robust_controller_chain[::-1]


def get_leaf_paths(G):

    leaves = []
    edge_paths = []

    for node in nx.nodes(G):
        is_leaf = True
        for edge in nx.edges(G):
            if edge[1] == node:
                is_leaf = False
                break
        if is_leaf:
            leaves.append(node)

    for leaf in leaves:
        node_path = list(nx.shortest_simple_paths(G, leaf, 0))[0]
        edge_paths.append([(s, t) for s, t in zip(node_path[:-1], node_path[1:])])
    return edge_paths


def get_robust_set_of_chains(C, tree, state_plan, goal_controller, verbose=False):

    edges = nx.edges(tree)

    ori_edge_plan = [(s.id, t.id) for s, t in zip(state_plan[1:], state_plan[:-1])]

    # initialize implicit controlsets with empty list
    implicit_ctrlsets = {edge: [] for edge in edges}
    nx.set_edge_attributes(tree, implicit_ctrlsets, "implicit_ctrlsets")

    grounded_ctrlsets = nx.get_edge_attributes(tree, "ctrlset")
    grounded_action = nx.get_edge_attributes(tree, "action")

    # iterate over each path, and get implicit conditions

    original_controllers = []
    for edge in reversed(ori_edge_plan):
        for name, con in grounded_ctrlsets[edge]:
            original_controllers.append((edge, name, con))

    implicit_chain = get_robust_chain(C, original_controllers, goal_controller, False)

    for edge, name, implicit_controller in implicit_chain:
        implicit_ctrlsets[edge].append((name, implicit_controller))

    set_of_chains = [implicit_chain]
    # go over every leaf path, and build implicit conditions for each
    for path in get_leaf_paths(tree):
        #path = get_leaf_paths(tree)[8]
        original_controllers = []
        print(path)
        sub_goal = goal_controller
        #go from bact to up
        for edge in path[::-1]:
            if len(implicit_ctrlsets[edge]) != 0:
                # need to take the first controller,
                sub_goal = implicit_ctrlsets[edge][0][-1]
                print(edge)
                for o in sub_goal.getObjectives():
                    print(o.feat().description(C))
            # else build
            else:
                for j, (name, con) in enumerate(grounded_ctrlsets[edge]):
                    original_controllers.insert(j, (edge, name, con))
        print("subgoal is:")
        for o in sub_goal.getObjectives():
            print(o.feat().description(C))
        partial_implicit_chain = get_robust_chain(C, original_controllers, sub_goal, verbose)
        for edge, name, implicit_controller in partial_implicit_chain:
            implicit_ctrlsets[edge].append((name, implicit_controller))

        # add entire path to set of control chains
        set_of_chains.append([(name, x) for edge in path for name, x in implicit_ctrlsets[edge]])

    nx.set_edge_attributes(tree, implicit_ctrlsets, "implicit_ctrlsets")

    return set_of_chains
