from pyddl import Domain, Problem, State, Action, neg, planner, backwards_planner
import actions
import predicates as pred
import util.domain_tower as dt
import libry as ry


"""
Simple planner to build a tower, which uses (simple) symbolic actions (STRIPS style)
"""
# TODO: add block order


def get_plan(verbose, control_actions, scene_obj, forward=False):

    # get simple action from all controllers
    domain = Domain((x.get_simple_action() for x in control_actions))

    # initial conditions
    init = list()

    # empty hand
    init_hand_empty = pred.HandEmpty("G")
    init_hand_empty.ground_predicate(G=scene_obj[dt.type_gripper][0])
    init.append(init_hand_empty.get_grounded_predicate())

    # free blocks
    for block in scene_obj[dt.type_block]:
        free_block = pred.BlockFree("B")
        free_block.ground_predicate(B=block)
        init.append(free_block.get_grounded_predicate())

    # normal goal predicates
    goal = list()

    # hand should be free
    goal_hand_empty = pred.HandEmpty("G")
    goal_hand_empty.ground_predicate(G=scene_obj[dt.type_gripper][0])
    goal.append(goal_hand_empty.get_grounded_predicate())

    # blocks on blocks
    for i in range(len(scene_obj[dt.type_block]) - 1):
        block_on_block = pred.BlockOnBlock("B", "B_placed")
        block_on_block.ground_predicate(B=scene_obj[dt.type_block][i], B_placed=scene_obj[dt.type_block][i+1])
        goal.append(block_on_block.get_grounded_predicate())

    print(goal)
    # define problem here
    prob = Problem(
        domain,
        scene_obj,
        init=init,
        goal=goal
    )

    G = None
    state_plan = None

    # generate plan
    if forward:
        plan = planner(prob, verbose=verbose)
    else:
        action_plan, state_plan, __ = backwards_planner(prob, goal=goal, action_tree=False, verbose=True)
        plan, state_plan, G = backwards_planner(prob, goal=goal, action_tree=True, max_diff=1,
                                                       root_state_plan=state_plan, verbose=True)
        # need to reverse plan
        plan = plan[::-1]
    if verbose:
        if plan is None:
            print('No Plan!')
        else:
            for action in plan:
                print(action)

    return plan, goal, state_plan, G


def get_goal_controller(C, goal):
    goal_feature = ry.CtrlSet()

    goals_block_on_block = [x for x in goal if x[0] == pred.BlockOnBlock.__name__]

    # cut out last goal, which was free hand
    for _, block, block_place_on in goals_block_on_block:

        goal_feature.addObjective(
            C.feature(ry.FS.positionRel, [block, block_place_on], [1e1], [0, 0, 0.1]),
            ry.OT.eq, -1)
        # should have z-axis in same direction
        # goal_feature.addObjective(
        #     C.feature(ry.FS.scalarProductZZ, [block, block_place_on], [1e-1], [1]),
        #     ry.OT.eq, -1)

        goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", block), True)

    return goal_feature


if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    action_list = [
        actions.GrabBlock(),
        actions.PlaceOn(),
    ]

    objects = {
             dt.type_block: (1, 2, 3),  # , "b3"),
             dt.type_gripper: ("gripper_R",)
         }

    # Parse arguments
    opts, args = parser.parse_args()
    get_plan(opts.verbose, action_list, objects)


