import libry as ry
import time
from util.setup_env import setup_tower_env

if __name__ == '__main__':

    # create real world and config with two blocks
    R, C, _ = setup_tower_env(3)
    C.view()
    tau = 0.01

    # approach the block
    approach = ry.CtrlSet()
    approach.addObjective(C.feature(ry.FS.positionRel, ["b1", "R_gripperCenter"], [1e1]), ry.OT.sos, .005)
    approach.addObjective(C.feature(ry.FS.scalarProductXZ, ["b1", "R_gripper"], [1e1]), ry.OT.sos, .01)

    # grasp block, use symbolic command
    grasp = ry.CtrlSet()
    grasp.addObjective(C.feature(ry.FS.aboveBox, ["b1", "R_gripperCenter"], [1e0]), ry.OT.ineq, -1)
    grasp_command = ["close_gripper", "R_gripper", "b1"]
    grasp.addSymbolicCommand(grasp_command, False)  # symbolic command to grasp object, Boolean if condition or run

    # place on othe rblock
    place_on = ry.CtrlSet()
    place_on.addSymbolicCommand(grasp_command, True)  # isCondition=True, therefore respected in initial feasibility
    place_on.addObjective(C.feature(ry.FS.scalarProductZZ, ["b2", "b1"], [1e1], [1]), ry.OT.sos, .005)
    place_on.addObjective(C.feature(ry.FS.positionRel, ["b2", "b1"], [1e1], [0, 0, -0.1]), ry.OT.sos, .005)

    open_gripper = ry.CtrlSet()
    open_command = ["open_gripper", "R_gripper", "b1"]
    open_gripper.addSymbolicCommand(open_command, False)  # isCondition=True, therefore respected in initial feasibility
    open_gripper.addObjective(C.feature(ry.FS.scalarProductZZ, ["b2", "b1"], [1e1], [1]), ry.OT.eq, -1)
    open_gripper.addObjective(C.feature(ry.FS.positionRel, ["b2", "b1"], [1e1], [0, 0, -0.1]), ry.OT.eq, -1)

    # move away, after placing block, to make sure opening gripper works
    move_up = ry.CtrlSet()
    move_up.addSymbolicCommand(open_command, True)
    move_up.addObjective(C.feature(ry.FS.positionRel, ["b3", "R_gripperCenter"], [1e1]), ry.OT.sos, .005)
    # only condition, should be solved through implicit conditions
    move_up.addObjective(C.feature(ry.FS.scalarProductZZ, ["b2", "b1"], [1e1], [1]), ry.OT.eq, -1)
    move_up.addObjective(C.feature(ry.FS.positionRel, ["b2", "b1"], [1e1], [0, 0, -0.1]), ry.OT.eq, -1)

    # it seems not all features work with ctrl solver, some get solved immediately!
    # following doesn't work, skips to end position
    # move_up.addObjective(C.feature(ry.FS.positionDiff, ["R_gripper", "b1"], [0, 0, 1e1], [0.3]*3), ry.OT.sos, .005)

    # simulation loop
    for t in range(0, 300):

        # currently need to rebuild the solver every step
        ctrl = ry.CtrlSolver(C, tau, 2)

        if move_up.canBeInitiated(C):
            ctrl.set(move_up)
        elif open_gripper.canBeInitiated(C):
            ctrl.set(open_gripper)
        elif place_on.canBeInitiated(C):
            ctrl.set(place_on)
        elif grasp.canBeInitiated(C):
            ctrl.set(grasp)
        elif approach.canBeInitiated(C):
            ctrl.set(approach)

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()

        time.sleep(tau)

    time.sleep(5)
    C.view_close()



