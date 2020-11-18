import controllers as con
from pyddl import Domain, Problem, State, Action, neg, planner
from util.setup_env import setup_tower_env

import libry as ry


def problem():

    #R, C = setup_tower_env(1)
    #C.view()
    # time.sleep(10)

    # two actions
    print("Action grasp")
    grasp = con.GraspBlock()
    grasp_action = grasp.get_action_pyddl()
    print(grasp_action.effects)

    print("Action place")
    place = con.PlaceOn()
    place_action = place.get_action_pyddl()

    close_gripper = con.CloseGripper()
    close_gripper_action = close_gripper.get_action_pyddl()

    domain = Domain((
        grasp_action,
        place_action,
        close_gripper_action
    ))

    objects = {
        "block": ("b1", "b2", "b3"),
        "gripper": "g"
    }

    type2sym = {
        str(ry.OT.eq): "=",
        str(ry.OT.ineq): "<=",
        str(ry.OT.sos): "=="
    }

    goal_obj_1 = con.Objective(
        FS=ry.FS.standingAbove,
        frames=["b1", "b2"],
        OT_type=ry.OT.eq,
        target=[1]*4,
        scale=[1]*4)

    goal_obj_2 = con.Objective(
        FS=ry.FS.standingAbove,
        frames=["b2", "b3"],
        OT_type=ry.OT.eq,
        target=[1] * 4,
        scale=[1] * 4)

    goal = (
        *goal_obj_1.get_pyddl_description(type2sym),
        *goal_obj_2.get_pyddl_description(type2sym),
    )

    ini_obj_1 = con.Objective(
        FS=ry.FS.standingAbove,
        frames=["b1", "b2"],
        OT_type=ry.OT.eq,
        target=[3] * 4,
        scale=[3] * 4)

    ini_obj_2 = con.Objective(
        FS=ry.FS.distance,
        frames=["b1", "g"],
        OT_type=ry.OT.eq,
        target=[3] * 1,
        scale=[1] * 1)

    ini_obj_3 = con.Objective(
        FS=ry.FS.distance,
        frames=["b2", "g"],
        OT_type=ry.OT.eq,
        target=[3] * 1,
        scale=[1] * 1)

    #print(*ini_obj_1.get_pyddl_description(type2sym))
    #print(*ini_obj_2.get_pyddl_description(type2sym))

    prob = Problem(
        domain,
        objects,
        init=(
            *ini_obj_1.get_pyddl_description(type2sym),
            #*ini_obj_2.get_pyddl_description(type2sym),
            #*ini_obj_3.get_pyddl_description(type2sym),
            #("=", (str(ry.FS.standingAbove), "b2", "b1"), 100),
            #("=", (str(ry.FS.vectorZDiff), "b1", "g"), 3),
            #("=", (str(ry.FS.vectorZDiff), "b2", "g"), 3)
        ),
        goal=goal
    )

    # generate plan
    plan = planner(prob, verbose=True)
    if plan is None:
        print('No Plan!')
    else:
        for action in plan:
            print(action)



if __name__ == '__main__':

    problem()