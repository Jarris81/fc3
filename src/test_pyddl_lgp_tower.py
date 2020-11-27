import controllers as con
from pyddl import Domain, Problem, State, Action, neg, planner
from util.setup_env import setup_tower_env
import time

import libry as ry


def problem(control_actions, scene_obj):

    # objects = {
    #     "block": ("b1", "b2"),  # , "b3"),
    #     "gripper": ("g",)
    # }

    for x in control_actions:
        print(x.name)
        y = x.get_action_pyddl(scene_obj)
        print(y.arg_names)
        print(f"Preconditions: {y.preconditions}")
        print(f"Effects: {y.effects}")
        print("")

    # get pyddl action from all controllers
    domain = Domain((x.get_action_pyddl(scene_obj) for x in control_actions))

    type2sym = {
        str(ry.OT.eq): "=",
        str(ry.OT.ineq): "<=",
        str(ry.OT.sos): "=="
    }

    goal_obj_1 = con.Objective(
        FS=ry.FS.standingAbove,
        frames=scene_obj["block"],
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
        #*goal_obj_1.get_pyddl_description(type2sym),
        #*goal_obj_2.get_pyddl_description(type2sym),
        ("grasping", scene_obj["block"][0], scene_obj["gripper"][0]),
    )

    prob = Problem(
        domain,
        scene_obj,
        init=(
            ("gripper_free", scene_obj["gripper"][0]),
        ),
        goal=goal
    )

    # generate plan
    plan = planner(prob, verbose=True)
    if plan is None:
        print('No Plan!')
    else:
        for action in plan:
            print(action.sig)

    return plan


def setup_scene():

    R, C = setup_tower_env(2)
    C.view()
    #time.sleep(10)

    return R, C


if __name__ == '__main__':

    control_actions = [
        con.Approach(),
        con.PlaceOn(),
        con.CloseGripper(),
        con.OpenGripper()
    ]

    name2con = {x.name: x for x in control_actions}

    _, C,  block_names = setup_tower_env(2)

    gripper_name = "R_gripperCenter"

    print("block names:", block_names)

    all_objects = {
             "block": block_names,  # , "b3"),
             "gripper": (gripper_name,)
         }

    plan = problem(control_actions, all_objects)

    C.view()
    tau = .01

    grounded_con = []

    # get grounded
    for action in plan:
        frames_real = action.sig[1:]
        con = name2con[action.sig[0]]
        x = con.get_grounded_control_set(C, frames_realized=frames_real)
        grounded_con.append((action, x))

    ctrl = ry.CtrlSolver(C, tau, 2)

    grounded_con = list(reversed(grounded_con))

    for t in range(0, 1000):

        for c in grounded_con:

            if c[1].canBeInitiated(ctrl):
                print(f"Initiating: {c[0]}")
                ctrl.set(c[1])
                break
            else:
                print(f"Cannot be initiated: {c[0]}")
                continue

        ctrl.update(C)
        q = ctrl.solve()
        C.setJointState(q)
        #C.computeCollisions()

        #     ctrl.report();
        #     C.watch(false, STRING(txt <<"t:" <<t));
        time.sleep(tau)





