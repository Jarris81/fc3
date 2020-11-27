import libry as ry
import time
from util.setup_env import setup_tower_env

if __name__ == '__main__':

    R, C, _= setup_tower_env(2)

    #C.setJointState(q)
    C.computeCollisions()

    C.view()

    #time.sleep(5)

    C.attach("R_gripper", "bb1")
    #C.update()

    ik = C.komo_path(1, 20, 10.0, True)

    aboveF = C.feature(ry.FS.positionRel, ["bb1", "bb2"], scale=[1, 1, 0])
    print(aboveF.eval(C))

    #ik.addObjective(times=[], feature=ry.FS.standingAbove, frames=["bb1", "bb2"], type=ry.OT.sos, scale=[1e1])

    ik.addObjective(times=[], feature=ry.FS.scalarProductZZ, frames=["bb2", "bb1"], type=ry.OT.eq, target=[1])
    ik.addObjective(times=[], feature=ry.FS.positionRel, frames=["bb2", "bb1"], type=ry.OT.sos, scale=[1e2], target=[0, 0, -0.1])

    # ik.addObjective(times=[], feature=ry.FS.distance, frames=["bb1", "bb2"], type=ry.OT.sos,
    #                 target=[0.2])
    # ik.addObjective(times=[], feature=ry.FS.vectorZ, frames=["bb1", "bb2"], type=ry.OT.sos)


    ik.optimize()
    ik.view(False, "result")
    #print(ik.getReport())
    #C.view()
    #print(ik.J())
    ik.view_play(0.5, False)
    #time.sleep(10)