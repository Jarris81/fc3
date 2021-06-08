import time
import sys
import libry as ry

import actions
import planners
from RLGS import RLGS
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


if __name__ == '__main__':

    C = None
    S = None
    action_list = []
    planner = None
    scene_objects = {}

    if "verbose" in sys.argv:
        add_verbose = True
    if "interference" in sys.argv:
        add_interference = True

    simulate = False
    cheat_rate = 50
    tau = 0.01

    if sys.argv[1] == "tower":
        C, S, scene_objects = setup_env.setup_tower_env(3)
        action_list = [
            actions.GrabBlock(),
            actions.PlaceOn(),
            actions.PlaceSide()
        ]
        planner = planners.TowerPlanner()

    elif sys.argv[1] == "pick_and_place":
        C, S, scene_objects = setup_env.setup_pick_and_place_env()
        action_list = [
            actions.GrabBlock(),
            actions.PlacePosition(),
        ]
        planner = planners.PickAndPlacePlanner()

    robot = RLGS(C, verbose=True)
    robot.setup(action_list, planner, scene_objects)

    for t in range(10000):
        if not t % cheat_rate and simulate:
            robot.cheat_update_obj(get_obj_info(S, C, scene_objects))

        # get the next q values of robot
        q = robot.step(t, tau)

        if simulate:
            S.step(q, tau, ry.ControlMode.position)

            # we need info from the simulation (or later real robot) if grasping worked
            gripper_action = robot.get_gripper_action()
            if gripper_action:
                if gripper_action.getCommand() == ry.SC.CLOSE_GRIPPER:
                    # close gripper until grasp was made, or failed
                    while not S.getGripperIsGrasping("R_gripper"):
                        S.closeGripper("R_gripper", speed=1.0)
                        S.step(q, tau, ry.ControlMode.position)
                        time.sleep(tau)
                elif gripper_action.getCommand() == ry.SC.OPEN_GRIPPER:
                    while S.getGripperIsGrasping("R_gripper") and S.getGripperWidth("R_gripper") < 0.2:
                        S.openGripper("R_gripper", speed=1.0)
                        S.step(q, tau, ry.ControlMode.position)
                        time.sleep(tau)

        time.sleep(tau)

        if robot.is_goal_fulfilled():
            break

    if robot.is_goal_fulfilled():
        print("Plan finished!")
    else:
        print("Plan not finished!")

    time.sleep(5)
