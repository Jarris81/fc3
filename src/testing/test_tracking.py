import libry as ry
import libpybot as pybot
import time
import actions
from util import setup_env, constants
import motioncapture
from robustness import get_robust_chain
from planners import TowerPlanner


if __name__ == '__main__':

    C, block_names = setup_env.setup_hand_over_env()

    C.view()

    lfinger1 = C.getFrame("L_panda_finger_joint2")
    rfingerjoin = C.getFrame("L_panda_finger_joint1")

    # for t in range(10):
    #     lfinger1.setRelativePosition((t / 100, 0, 0))
    #     rfingerjoin.setRelativePosition((t / 100, 0, 0))
    #     time.sleep(0.1)

    eqPrecision = 1e-2

    tau = 0.1

    gripper_take, gripper_give = "L_gripper", "R_gripper"

    mc = motioncapture.MotionCaptureOptitrack("130.149.82.29")

    for i in range(10000):

        # mc = motioncapture.connect("optitrack", "130.149.82.29")
        while True:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                # print(name, obj.position, obj.rotation.z)
                # print(name)
                if name == "b1":
                    pos = list(obj.position)
                    pos[2] += 0.68
                    f = C.frame("b1")
                    f.setPosition(pos)
                    f.setQuaternion((obj.rotation.w, obj.rotation.x, obj.rotation.y, obj.rotation.z))
                    # print("setting position")

            # time.sleep(0.5)
