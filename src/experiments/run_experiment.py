import time
from optparse import OptionParser

import actions
import planners
from execution_models import RLGS, SimpleSystem, RLDSClone
from interference import ResetPosition, NoInterference
import util.setup_env as setup_env

from tracking import Tracker

"""
Build a tower with the provided plan. 
- During execution, a block is placed to its original position, which should show 
interference in the real world. 
- we want to show that we 
"""


def run_experiment(model_name, experiment_name, interference_num, use_real_robot, tracking, verbose=False):
    C = None
    action_list = []
    planner = None
    scene_objects = {}
    tau = 0.01
    interference_list = [NoInterference()]

    add_verbose = False
    add_interference = True

    tracker = None

    #
    # Tower Experiment
    #
    if experiment_name == "tower":

        C, scene_objects = setup_env.setup_tower_env()
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
            ResetPosition(190, 192, "b2", ori_pos_b2),
            # b2 is knocked of tower while gripper is holding b1
            ResetPosition(150, 262, "b2", ori_pos_b2),
            # b1 is moved out of reach
            ResetPosition(50, 52, "b1", infeasible_pos_b1)
        ))

    #
    # Hand Over
    #
    elif experiment_name == "hand_over":
        C, scene_objects = setup_env.setup_hand_over_env()
        action_list = [
            actions.GrabBlock(),
            actions.PlaceGoal(),
            actions.HandOver()
        ]
        planner = planners.HandOverPlanner()
        x_flip_pos_b1 = C.frame("b1").getPosition()
        x_flip_pos_b1[0] = x_flip_pos_b1[0] * -1
        interference_list.extend((
            # b1 is knocked of tower while gripper is moving to b1
            ResetPosition(5, 7, "b1", x_flip_pos_b1),
        ))

        tracker = Tracker(C, ["b1"], 1) if tracking else None
    #
    # Stick Pull
    #
    elif experiment_name == "stick_pull":
        C, scene_objects = setup_env.setup_stick_pull_env()
        action_list = [
            actions.GrabBlock(),
            actions.PlaceGoal(),
            actions.GrabStick(),
            actions.PullBlockToGoal(),
            actions.PlaceStick()
        ]
        planner = planners.StickPullPlanner()
        x_new_pos_b1 = C.frame("b1").getPosition()
        x_new_pos_b1[1] += 0.3
        interference_list.extend((
            # b1 is knocked of tower while gripper is moving to b1
            ResetPosition(30, 32, "b1", x_new_pos_b1),
        ))

        tracker = Tracker(C, ["b1", "stick"], 1) if tracking else None

    #
    # Bottle Open
    #
    elif experiment_name == "bottle_open":
        C, scene_objects = setup_env.setup_bottle_open_env()
        action_list = [

        ]

    current_interference = interference_list[interference_num]

    exec_model = None
    if model_name == "rlgs":
        exec_model = RLGS(C, verbose=verbose, use_real_robot=use_real_robot)
    elif model_name == "simple":
        exec_model = SimpleSystem(C, verbose=verbose, use_real_robot=use_real_robot)
    elif model_name == "rlds_clone":
        exec_model = RLDSClone(C, verbose=verbose, use_real_robot=use_real_robot)

    # Tracking Setup if specified
    tracker = Tracker(C,
                      [x for y in scene_objects.values() for x in y],
                      1) \
        if tracking else None

    if tracker:
        tracker.update(0)

    C.view()

    if not exec_model.setup(action_list, planner, scene_objects):
        print("Plan is not feasible!")
        C.view_close()
        return

    # run do the task
    exec_model.run()

    if exec_model.is_goal_fulfilled():
        print("Plan finished!")
    else:
        print("Plan not finished!")

    exec_model.move_home()

    time.sleep(5)
    C.view_close()


if __name__ == '__main__':
    choices_model = ["simple", "rlgs", "rlds_clone"]
    choices_scenario = ["tower", "hand_over", "stick"]

    parser = OptionParser()

    parser.add_option("-m", "--model", dest="model_name", default="rlgs",
                      help="Which system to use", type="choice", choices=choices_model)

    parser.add_option("-v", "--verbose", dest="verbose", default=False, action="store_true",
                      help="don't print status messages to stdout")

    parser.add_option("-s", "--scenario", dest="experiment_name",
                      help="Define the experiment", type="choice", choices=choices_scenario)

    parser.add_option("-i", "--interference", dest="interference_num", default=0,
                      help="Define interference", type="int")

    parser.add_option("-r", "--use_real_robot", dest="use_real_robot", default=False, action="store_true",
                      help="use real robots or simulate")

    parser.add_option("-t", "--tracking", dest="tracking", default=False, action="store_true",
                      help="use tracking with Optitrack")

    (options, args) = parser.parse_args()

    run_experiment(model_name=options.model_name,
                   experiment_name=options.experiment_name,
                   interference_num=options.interference_num,
                   use_real_robot=options.use_real_robot,
                   tracking=options.tracking,
                   verbose=options.verbose)
