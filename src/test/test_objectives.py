import libry as ry
import time
from util.setup_env import setup_tower_env

if __name__ == '__main__':

    R, C, _ = setup_tower_env(2)

    #C.setJointState(q)
    #C.computeCollisions()

    C.view()

    tau = 0.01



    C.computeCollisions()
    #C.checkView()

    #C.attach("R_gripper", "b1")

    b1 = C.getFrame("b1")
    print(b1.info())

    C.computeCollisions()

    solver = ry.CtrlSolver(C, tau, 2)

    grasp = ry.CtrlSet()
    move = ry.CtrlSet()
    #grasp.addObjective(C.feature(ry.FS.positionRel, ["b1", "R_gripperCenter"], [1e1], [.0, 0., -.15]), ry.OT.sos, .005)

    a = C.feature(ry.FS.positionRel, ["b1", "R_gripperCenter"], [1e1], [.0, 0., -.15])

    print(a.eval(C))
    # print(grasp)
    # #print(a)
    #
    command = ["grasp", "run", "R_gripper", "b1"]
    command1 = ["grasp", "init", "R_gripper", "b1"]
    grasp.addSymbolicCommand(command)
    move.addSymbolicCommand(command1)


    init_grasp = grasp.canBeInitiated(C)
    init_move = move.canBeInitiated(C)
    print("is grasp init: ", init_grasp)
    print("is move init: ", init_move)

    # actually do the movement
    print("Do update...")
    solver.set(grasp)
    solver.update(C)

    init_grasp = grasp.canBeInitiated(C)
    init_move = move.canBeInitiated(C)
    print("is grasp init: ", init_grasp)
    print("is move init: ", init_move)

    b1 = C.getFrame("b1")
    print(b1.info())


    # print(C.getFrame("R_gripper").info())
    #
    #
    #
    # #time.sleep(5)
    #
    # print(C.getJointNames())
    # print(C.getJointState())

    #S = C.simulation(ry.SimulatorEngine.physx, True)
    #C.setCloseGripper("L_gripper", 0.2)





    #for i in range(100):
        #C.setCloseGripper("L_gripper", 0.02)
        #C.computeCollisions()
        #time.sleep(tau)

    # C.attach("R_gripper", "b1")
    # #S.closeGripper("R_gripper")
    #
    # print(C.getJointNames())
    # print(C.getJointState())
    #
    # ik = C.komo_path(1, 20, 10.0, True)
    #
    # aboveF = C.feature(ry.FS.positionRel, ["b1", "b2"], scale=[1, 1, 0])
    # print(aboveF.eval(C))
    #
    # #ik.addObjective(times=[], feature=ry.FS.stary.SimulatorEngine.physx, TruendingAbove, frames=["bb1", "bb2"], type=ry.OT.sos, scale=[1e1])
    #
    # ik.addObjective(times=[], feature=ry.FS.scalarProductZZ, frames=["b2", "b1"], type=ry.OT.eq, target=[1])
    # ik.addObjective(times=[], feature=ry.FS.positionRel, frames=["b2", "b1"], type=ry.OT.sos, scale=[1e2], target=[0, 0, -0.1])
    #
    # # ik.addObjective(times=[], feature=ry.FS.distance, frames=["bb1", "bb2"], type=ry.OT.sos,
    # #                 target=[0.2])
    # # ik.addObjective(times=[], feature=ry.FS.vectorZ, frames=["bb1", "bb2"], type=ry.OT.sos)
    #
    #
    #
    #
    # ik.optimize()
    # ik.view(False, "result")
    # print(ik.getReport())
    # C.view()
    # #print(ik.J())
    # ik.view_play(0.5, False)
    # time.sleep(10)