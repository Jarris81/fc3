from pyddl import Domain, Problem, State, Action, neg, planner
import actions
import util.domain_tower as dt
import libry as ry

"""
Simple planner to build a tower, which uses (simple) symbolic actions (STRIPS style)
"""
# TODO: add block order


def get_plan(verbose, control_actions, scene_obj):

    # get simple action from all controllers
    domain = Domain((x.get_simple_action(scene_obj) for x in control_actions))

    # goal is for now numerical order of block placed on each other, unless specified otherwise
    goal = [(dt.b_on_b, scene_obj[dt.type_block][i], scene_obj[dt.type_block][i + 1])\
            for i in range(len(scene_obj[dt.type_block]) - 1)]
    # also append free hand
    goal.append((dt.hand_empty, scene_obj[dt.type_gripper][0]))

    print(goal)

    # normal initial conditions
    init_free_hand = (dt.hand_empty, scene_obj[dt.type_gripper][0])
    init_free_blocks = [(dt.block_free, block) for block in scene_obj[dt.type_block]]

    # extend all initial conditions
    init = []
    init.append(init_free_hand)
    init.extend(init_free_blocks)

    # define problem here
    prob = Problem(
        domain,
        scene_obj,
        init=init,
        goal=goal
    )

    # generate plan
    plan = planner(prob, verbose=verbose)
    if verbose:
        if plan is None:
            print('No Plan!')
        else:
            for action in plan:
                print(action)

    return plan, goal


def get_goal_controller(C, goal):
    goal_feature = ry.CtrlSet()

    # cut out last goal, which was free hand
    for _, block, block_place_on in goal[:-1]:

        goal_feature.addObjective(
            C.feature(ry.FS.positionRel, [block, block_place_on], [1e1], [0, 0, 0.105]),
            ry.OT.eq, -1)
        # should have z-axis in same direction
        goal_feature.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, block_place_on], [1e1], [1]),
            ry.OT.eq, -1)

        goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", block), True)

    return goal_feature


if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    actions = [
        actions.ApproachBlock(),
        actions.PlaceOn(),
        #actions.CloseGripper(),
        #actions.OpenGripper()
    ]

    objects = {
             dt.type_block: (1, 2, 3),  # , "b3"),
             dt.type_gripper: ("gripper_R",)
         }

    # Parse arguments
    opts, args = parser.parse_args()
    get_plan(opts.verbose, actions, objects)


