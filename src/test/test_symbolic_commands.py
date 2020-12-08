import libry as ry
import time
from util.setup_env import setup_tower_env

if __name__ == '__main__':

    R, C, _ = setup_tower_env(2)
    C.view()

    tau = 0.01

    ctrl = ry.CtrlSolver(C, tau, 2)
    approach = ry.CtrlSet()

    test = ry.CtrlObjective()

    #approach.addObjective(C.feature(ry.FS.vectorZDiff, ["b1", "R_gripperCenter"], [1e1]), ry.OT.sos, .005)
    approach.addObjective(C.feature(ry.FS.positionRel, ["b1", "R_gripperCenter"], [1e1]), ry.OT.sos, .005)

    print(type(test.get_OT()))
    print(approach.getObjectives()[0])

    grasp = ry.CtrlSet()
    grasp.addObjective(C.feature(ry.FS.aboveBox, ["b1", "R_gripperCenter"], [1e0]), ry.OT.ineq, -1)
    command = ["grasp", "R_gripper", "b1"]
    grasp.addSymbolicCommand(command, False)

    move_up = ry.CtrlSet()
    command1 = ["grasp", "R_gripper", "b1"]
    move_up.addSymbolicCommand(command1, True)

    # This is working with "R_gripper", but also should work only with b1
    # solver does not have correct Config
    move_up.addObjective(C.feature(ry.FS.position, ["R_gripper"], [1e1], [0.1, 0.1, 1],), ry.OT.sos, .005)

    # doesnt work, since C not up-to-date
    #move_up.addObjective(C.feature(ry.FS.scalarProductZZ, ["b2", "b1"], [1e1]), ry.OT.sos, .005)
    #move_up.addObjective(C.feature(ry.FS.positionRel, ["b2", "b1"], [1e1], [0, 0, -0.1]), ry.OT.sos, .005)

    for t in range(0, 1000):

        if move_up.canBeInitiated(C):
            ctrl.set(move_up)
            print("MOVE UP")
        elif grasp.canBeInitiated(C):
            ctrl.set(grasp)
            print("GRASP")
        elif approach.canBeInitiated(C):
            ctrl.set(approach)
            print("APPROACH")

        ctrl.update(C)
        q = ctrl.solve()
        C.setJointState(q)
        C.computeCollisions()

        time.sleep(.01)


