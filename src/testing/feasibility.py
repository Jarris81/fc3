import libry as ry
import time
from util.setup_env import setup_tower_env

'''
- only need to set switches when frames get different parent frames
- for my case, thats when we grasp or release obj/blocks
'''

if __name__ == '__main__':

    R, C, _ = setup_tower_env(1)
    C.view()

    tau = 0.01


    komo = ry.KOMO()
    komo.setModel(C, False)
    komo.setTiming(3., 10, 5., 1)  # DIFFERENT

    komo.add_qControlObjective([], 1, 1e-1)  # DIFFERENT
    #komo.addSquaredQuaternionNorms([], 3.)

    # grasp
    komo.addSwitch_stable(1., 2., "table", "R_gripper", "b1")
    komo.addObjective([1.], ry.FS.positionDiff, ["R_gripperCenter", "b1"], ry.OT.eq, [1e2])
    komo.addObjective([1.], ry.FS.scalarProductXX, ["R_gripper", "b1"], ry.OT.eq, [1e2], [0.])
    komo.addObjective([1.], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    # lift up
    #komo.addSwitch_stable(2., 3., "R_gripper", "R_gripper", "b1")
    komo.addObjective([1.5], ry.FS.position, ["R_gripper"], ry.OT.eq, [1e2], [0, 0.2, 1])
    komo.addObjective([1.5], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    # place
    komo.addSwitch_stable(2., -1., "R_gripper", "table", "b1")
    komo.addObjective([2.], ry.FS.positionDiff, ["b1", "table"], ry.OT.eq, [1e2], [0.2, 0, .08])
    komo.addObjective([2.], ry.FS.vectorZ, ["R_gripper"], ry.OT.eq, [1e2], [0., 0., 1.])

    komo.optimize()
    komo.view(False, "result")
    print("constraints:", komo.getConstraintViolations())
    print("costs:", komo.getCosts())
    komo.getReport()

    komo.view_play(0.5, False)

    time.sleep(5)