import libry as ry
import time
import actions
from util.setup_env import setup_tower_env, setup_pick_and_place_env


if __name__ == '__main__':

    C, S, block_names = setup_pick_and_place_env()

    grab_stick = actions.GrabStick()
    pull_block = actions.PullBlockStick()

    C.view()

    tau = 0.01

    robust_plan = []
    robust_plan.extend(grab_stick.get_grounded_control_set(C, ["R_gripper", "stick"]))
    robust_plan.extend(pull_block.get_grounded_control_set(C, ["R_gripper", "b1", "stick"]))

    for t in range(0, 10000):

        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, c in enumerate(robust_plan):
            if c.canBeInitiated(C):
                ctrl.set(c)

            # update simulation/ make a step
            ctrl.update(C)
            q = ctrl.solve(C)
            C.setJointState(q)
            C.computeCollisions()
            coll = C.getCollisions(0)
            time.sleep(tau)
