from pyddl import Domain, Problem, State, Action, neg, planner
import controllers_geo as con
import util.domain_tower as dt
from src.util.setup_env import setup_tower_env
import numpy as np
import time

"""
Test for a simple planner to build a tower. 
"""


def problem(verbose, control_actions, scene_obj):

    # declare actions here
    # TODO add more actions to match real scenario
    # TODO: Add implicit conditions

    for x in control_actions:
        print(x.name)
        y = x.get_simple_action(scene_obj)
        #print(y.arg_names)
        print(f"Preconditions: {y.preconditions}")
        print(f"Effects: {y.effects}")
        print("")

    # get simple action from all controllers
    domain = Domain((x.get_simple_action(scene_obj) for x in control_actions))

    # define goal
    goal = (
            (dt.b_on_b, 1, 2),
            (dt.b_on_b, 2, 3),
    )

    # normal initial conditions
    init = [(dt.block_free, block) for block in scene_obj[dt.type_block]]
    init.append((dt.hand_empty, scene_obj[dt.type_gripper][0]))

    # define problem here
    prob = Problem(
        domain,
        scene_obj,
        init=init,
        goal=goal
    )

    #generate plan
    plan = planner(prob, verbose=verbose)
    if plan is None:
        print('No Plan!')
    else:
        for action in plan:
            print(action)


if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    actions = [
        con.Approach(),
        con.PlaceOn(),
        con.CloseGripper(),
        con.OpenGripper()
    ]

    objects = {
             dt.type_block: (1, 2, 3),  # , "b3"),
             dt.type_gripper: ("gripper_R",)
         }

    # Parse arguments
    opts, args = parser.parse_args()
    problem(opts.verbose, actions, objects)
