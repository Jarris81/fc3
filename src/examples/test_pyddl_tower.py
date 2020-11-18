from pyddl import Domain, Problem, State, Action, neg, planner
from src.util.setup_env import setup_tower_env
import numpy as np
import time


def problem(verbose):

    pick_up = Action("pick-up",
               parameters=(
                   ("block", "b"),
                   ("gripper", "g")
               ),
               preconditions=(
                   ("free", "b"),
                   ("empty", "g")

               ),
               effects=(
                   ("inhand", "g", "b"),
                   neg(("free", "b")),
                   neg(("empty", "g"))
               ))
    place_on = Action("place_on",
           parameters=(
               ("block", "d"),
               ("gripper", "g"),
               ("block", "c")
           ),
           preconditions=(
               ("inhand", "g", "d"),
               ("free", "c")
           ),
           effects=(
               neg(("inhand", "g", "d")),
               neg(("free", "c")),
               ("free", "d"),
               ("empty", "g"),
               ("on", "d", "c")

           ))



    # define domain here (possible actions)
    domain = Domain((
        pick_up,
        place_on
    ))

    goal =(
            ("on", 1, 2),
            ("on", 2, 3),
    )

    # define problem here
    prob = Problem(
        domain,
        {
            'block': (1, 2, 3),
            "gripper": "g"
        },
        init=(
            ("empty", "g"),
            ("free", 1),
            ("free", 2),
            ("free", 3)
        ),
        goal=goal
    )

    #generate plan
    plan = planner(prob, verbose=verbose)
    if plan is None:
        print('No Plan!')
    else:
        print(type(plan))
        for action in plan:
            print(action)

    ## need to calculate the end effect of actions

    for action in plan:
        predicates = list()
        functions = dict()
        for predicate in action.preconditions:
            if predicate[0] == '=':
                functions[predicate[1]] = predicate[2]
            else:
                predicates.append(predicate)
        temp_state = State(predicates, functions)
        final_state = temp_state.apply(action)
        #print("Start_cond", temp_state.predicates)
        #print("final_cond", final_state.predicates)

    def get_implicit_feasibility(plan, goal):

        implied_cond = []

        #plan.append(goal)
        for i in reversed(range(0, len(plan))):
            # check if we are at goal:
            action = plan[i]
            if i == len(plan) - 1:
                implied_cond_i = [x for x in plan if x not in action.preconditions]
            #else:
            # for each action, get the name
            print(action.preconditions)


    get_implicit_feasibility(plan, goal)
    # get the implicit feasibility


if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    # Parse arguments
    opts, args = parser.parse_args()
    problem(opts.verbose)