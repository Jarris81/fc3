import controllers as con
from pyddl import Domain, Problem, State, Action, neg, planner
from util.setup_env import setup_tower_env

import libry as ry


def problem():

    #R, C = setup_tower_env(1)
    #C.view()
    # time.sleep(10)

    # two actions
    grasp = con.GraspBlock()
    grasp_action = grasp.get_action_pyddl()

    print("Action place")
    place = con.PlaceOn()
    place_action = place.get_action_pyddl()

    domain = Domain((
        grasp_action,
        place_action
    ))

    objects = {
        "block": (1, 2),
        "gripper": "g"
    }

    goal = (
        (str(ry.FS.standingAbove), 1, 2),
    )

    prob = Problem(
        domain,
        objects,
        init=(),
        goal=goal
    )

    # generate plan
    plan = planner(prob)
    if plan is None:
        print('No Plan!')
    else:
        print(type(plan))
        for action in plan:
            print(action)








if __name__ == '__main__':

    problem()