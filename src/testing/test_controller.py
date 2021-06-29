import libry as ry
import time
import actions
from util.setup_env import setup_tower_env, setup_pick_and_place_env


if __name__ == '__main__':

    C, block_names = setup_pick_and_place_env()

    grab_stick = actions.GrabStick()
    pull_block = actions.PullBlockStick()

    grab_block = actions.GrabBlock()
    handover = actions.HandOver()
    move_to = actions.PlacePosition()

    C.view()

    tau = 0.01

    robust_plan = []
    # robust_plan.extend(grab_stick.get_grounded_control_set(C, ["R_gripper", "stick"]))
    # robust_plan.extend(pull_block.get_grounded_control_set(C, ["R_gripper", "b1", "stick"]))

    robust_plan.extend(grab_block.get_grounded_control_set(C, ["R_gripper", "b1"]))
    robust_plan.extend(handover.get_grounded_control_set(C, ["R_gripper", "L_gripper", "b1"]))

    for a in robust_plan:
        a.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e2]), ry.OT.ineq)


    for t in range(0, 10000):

        q_real = C.getJointState()
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, c in enumerate(robust_plan[::-1]):
            if c.canBeInitiated(ctrl):
                ctrl.set(c)
                break
            # update simulation/ make a step
        ctrl.update(q_real, [], C)
        q = ctrl.solve()
        C.setJointState(q)
        C.computeCollisions()
        #coll = C.getCollisions(0)
        time.sleep(tau)
