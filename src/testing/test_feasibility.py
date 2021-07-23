import libry as ry
import time
from util.setup_env import setup_tower_env
import actions
import feasibility

from planners import TowerPlanner


'''
- only need to set switches when frames get different parent frames
- for my case, thats when we grasp or release obj/blocks
'''

if __name__ == '__main__':

    C, scene_objects = setup_tower_env()

    tau = 0.01

    # grab_block = actions.GrabBlock()
    # place_block_place = actions.PlaceOn()
    #
    # robust_plan = []
    # robust_plan.extend(grab_block.get_grounded_control_set(C, ["R_gripper", "b1"]))
    # robust_plan.extend(place_block_place.get_grounded_control_set(C, ["R_gripper", "b1", "b2"]))
    #
    # planner = TowerPlanner()
    # goal = planner.get_goal_controller(C)
    #
    #
    # feasibility.check_switch_chain_feasibility(C, robust_plan, goal, scene_objects, verbose=True)

    # get gripper position and pose
    gripper_f = C.getFrame("R_gripper")
    box_f = C.frame("b1")

    x = gripper_f.getPosition()
    C.attach("R_gripper", "b1")

    box_f.setPosition(x)

    komo = ry.KOMO()
    komo.setModel(C, False)
    komo.setTiming(4., 1, 5., 2)  # DIFFERENT


    komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    komo.addSquaredQuaternionNorms([], 3.)

    # grasp
    komo.addSwitch_stable(1., 2., "table", "R_gripper", "b1")
    komo.addObjective([1.], ry.FS.positionDiff, ["R_gripperCenter", "b1"], ry.OT.eq, [1e2])
    komo.addObjective([1.], ry.FS.scalarProductXX, ["R_gripper", "b1"], ry.OT.eq, [1e2], [0.])
    komo.addObjective([1.], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    # lift up
    #komo.addSwitch_stable(2., 3., "R_gripper", "R_gripper", "b1")
    # komo.addObjective([1.5], ry.FS.position, ["R_gripper"], ry.OT.eq, [1e1], [0, 0.2, 1])
    # komo.addObjective([1.5], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    # move
    #komo.addSwitch_stable(2., -1., "R_gripper", "table", "b1")
    komo.addObjective([1.], ry.FS.positionDiff, ["R_gripper", "table"], ry.OT.eq, [1e1], [0.2, 0, .5])
    komo.addObjective([1.], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e1], [0., 0., 1.])

    # move
    komo.addSwitch_stable(1., 2., "R_gripper", "table", "b1")
    komo.addObjective([2.], ry.FS.positionDiff, ["b1", "table"], ry.OT.eq, [1e1], [0.2, 0, .08])
    komo.addObjective([2.], ry.FS.vectorZ, ["b1"], ry.OT.eq, [1e1], [0., 0., 1.])

    #grasp
    komo.addSwitch_stable(2., 3., "table", "R_gripper", "b1")
    komo.addObjective([3.], ry.FS.positionDiff, ["R_gripperCenter", "b1"], ry.OT.eq, [1e2])
    komo.addObjective([3.], ry.FS.scalarProductXX, ["R_gripper", "b1"], ry.OT.eq, [1e2], [0.])
    komo.addObjective([3.], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    # lift up
    # komo.addSwitch_stable(2., 3., "R_gripper", "R_gripper", "b1")
    komo.addObjective([4], ry.FS.position, ["R_gripper"], ry.OT.eq, [1e1], [0, 0.2, 1])
    komo.addObjective([4], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    komo.optimize()
    komo.view(False, "result")
    print("constraints:", komo.getConstraintViolations())
    print("costs:", komo.getCosts())
    komo.getReport(True)

    komo.view_play(1, False)

    time.sleep(5)