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

from tracking import Tracker

import libpybot as pybot

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


def run_experiment(experiment_name, interference_num, use_config_only, use_real_robot, tracking, verbose=False):
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


    # Tracking Setup if specified
    tracker = Tracker(C,
                      [x for y in scene_objects.values() for x in y],
                      1) \
        if tracking else None

    if tracker:
        tracker.update(0)

    C.view()

    rlgs = RLGS(C, verbose=False)
    if not rlgs.setup(action_list, planner, scene_objects):
        print("Plan is not feasible!")
        C.view_close()
        return

    # Setup the real or simulated robot here
    if not use_config_only:

        bot = pybot.BotOp(C, use_real_robot, "BOTH", "ROBOTIQ")
    #     bot.home(C)
    #     while bot.getTimeToEnd() > 0:
    #         bot.step(C, .1)
    #         time.sleep(.1)
    #
        bot.gripperOpen("RIGHT", 1, 1)
        while not bot.gripperDone("RIGHT"):
            print("gripper is not done")
            time.sleep(0.1)

    # Loop
    for t in range(10000):
        if tracker:
            tracker.update(t)

        if add_interference:
            current_interference.do_interference(C, t)

        # get the next q values of robot
        q, gripper_action = rlgs.step(t, tau)

        if not use_config_only:

            if gripper_action is True:
                bot.gripperClose("RIGHT", 0.5, 0.01, 0.1)

                while not bot.gripperDone("RIGHT"):
                    time.sleep(0.1)

            elif gripper_action is False:
                bot.gripperOpen("RIGHT", 1, 0.1)

                while not bot.gripperDone("RIGHT"):
                    time.sleep(0.1)

            # move the real bot
            bot.moveLeap(q, 2)

            # update config
            bot.step(C, 0.1)
        else:
            C.setJointState(q)
            time.sleep(tau)

        if rlgs.is_goal_fulfilled() or rlgs.is_no_plan_feasible():
            break

    if rlgs.is_no_plan_feasible():
        print("No Plan is feasible, abort")
    elif rlgs.is_goal_fulfilled():
        print("Plan finished!")
    else:
        print("Plan not finished!")

    if not use_config_only:
        # move the robot home
        bot.home(C)
        while bot.getTimeToEnd() > 0:
            bot.step(C, 0)

    else:
        while not rlgs.is_home():
            q = rlgs.move_home()
            C.setJointState(q)
            time.sleep(tau)

    time.sleep(5)
    C.view_close()


if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-v", "--verbose", dest="verbose", default=False,
                      help="don't print status messages to stdout")

    parser.add_option("-s", "--scenario", dest="experiment_name", default=True,
                      help="Define the experiment")

    parser.add_option("-i", "--interference", dest="interference_num", default=0,
                      help="Define interference")

    parser.add_option("-m", "--run_mode", dest="run_mode", default="config",
                      help="run mode: config, sim, or real")

    parser.add_option("-t", "--tracking", dest="tracking", default=False,
                      help="use tracking with Optitrack")

    (options, args) = parser.parse_args()

    option_config_only = True
    option_real_robot = False

    if options.run_mode == "sim":
        option_config_only = False
    elif options.run_mode == "real":
        option_config_only = False
        option_real_robot = True

    run_experiment(options.experiment_name,
                   interference_num=int(options.interference_num),
                   use_config_only=option_config_only,
                   use_real_robot=option_real_robot,
                   tracking=options.tracking,
                   verbose=options.verbose)
