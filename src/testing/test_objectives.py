import libry as ry
import time
from util.setup_env import setup_tower_env


if __name__ == '__main__':

    R, C, _ = setup_tower_env(3)

    #C.setJointState(q)
    #C.computeCollisions()

    C.view()

    tau = 0.01

    time.sleep(1)

    komo = ry.KOMO()
    komo.setModel(C, False)

    komo.setTiming(1, 10, 5., 2)
    komo.add_qControlObjective([], 2)
    komo.addSquaredQuaternionNorms([], 3.)

    # Gripper Center close as possible
    komo.addObjective([1], ry.FS.positionDiff, ["R_gripperCenter", "b1"], ry.OT.sos, [1e1]*3)

    # gripper should z axis should align with block (top grasp)
    komo.addObjective([1], ry.FS.vectorZDiff, ["R_gripper", "b1"], ry.OT.sos, [1e2])

    #
    komo.addObjective([], ry.FS.distance, ["R_gripper", "b1"], ry.OT.ineq, [1e1])

    komo.optimize()

    komo.view(False, "result")

    time.sleep(5)

    komo.view_play(.2, False)

    time.sleep(5)
    report = komo.getReport(True)



