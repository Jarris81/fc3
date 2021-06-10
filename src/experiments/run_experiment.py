import time
import sys
import libry as ry
import random
from optparse import OptionParser


import actions
import planners
from RLGS import RLGS
from interference import ResetPosition, NoInterference
import util.setup_env as setup_env




"""
Build a tower with the provided plan. 
- During execution, a block is placed to its original position, which should show 
interference in the real world. 
- we want to show that we 
"""


def get_obj_info(S, C, scene_objects):

    obj_infos = {}

    all_objects = [x for y in scene_objects for x in scene_objects[y]]
    for block in all_objects:
        if "gripper" in block:
            continue
        shape = C.frame(block).info()["shape"]
        info = {"pos": S.getGroundTruthPosition(block),
                "size": S.getGroundTruthSize(block),
                "rot_max": S.getGroundTruthRotationMatrix(block),
                "shape": shape}
        obj_infos[block] = info

    return obj_infos


def run_experiment(experiment_name, interference_num=0, verbose=False):

    C = None
    action_list = []
    planner = None
    scene_objects = {}

    add_verbose = False
    add_interference = True

    interference_list = [NoInterference()]

    tau = 0.01

    if experiment_name == "tower":
        C, scene_objects = setup_env.setup_tower_env(3)
        action_list = [
            actions.GrabBlock(),
            actions.PlaceOn(),
            actions.PlaceSide()
        ]
        planner = planners.TowerPlanner()

        # add interference
        ori_pos_b2 = C.frame("b2").getPosition()
        infeasible_pos_b1 = (0.6, 0.6, 0.68)

        interference_list.extend((
            # b2 is knocked of tower while gripper is moving to b1
            ResetPosition(300, 302, "b2", ori_pos_b2),
            # b2 is knocked of tower while gripper is holding b1
            ResetPosition(450, 452, "b2", ori_pos_b2),
            # b1 is moved out of reach
            ResetPosition(50, 52, "b1", infeasible_pos_b1)
        ))

    elif experiment_name == "pick_and_place":
        C, scene_objects = setup_env.setup_pick_and_place_env()
        action_list = [
            actions.GrabBlock(),
            actions.PlacePosition(),
        ]
        planner = planners.PickAndPlacePlanner()

    current_interference = interference_list[interference_num]
    C.view()

    robot = RLGS(C, verbose=verbose)
    if not robot.setup(action_list, planner, scene_objects):
        print("Plan is not feasible!")
        return

    for t in range(10000):
        if add_interference:
            current_interference.do_interference(C, t)

        # get the next q values of robot
        q = robot.step(t, tau)

        time.sleep(tau)

        if robot.is_goal_fulfilled() or robot.is_no_plan_feasible():
            break

    if robot.is_no_plan_feasible():
        print("No Plan is feasible, abort")
    elif robot.is_goal_fulfilled():
        print("Plan finished!")
    else:
        print("Plan not finished!")

    time.sleep(5)
    C.view_close()



if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-q", "--quiet",
                      action="store_false", dest="verbose", default=False,
                      help="don't print status messages to stdout")

    parser.add_option("-s", "--scenario", dest="experiment_name", default=True,
                      help="Define the experiment")

    parser.add_option("-i", "--interference", dest="interference_num", default=0,
                      help="Define interference")

    (options, args) = parser.parse_args()
    run_experiment(options.experiment_name,
                   int(options.interference_num),
                   options.verbose)


