import libry as ry
import libpybot as pybot
import time
import actions
from util import setup_env
from robustness import get_robust_chain
from planners import TowerPlanner

if __name__ == '__main__':

    C, block_names = setup_env.setup_bottle_open_env()

    grab_stick = actions.GrabStick()
    pull_block = actions.PullBlockStick()

    grab_block = actions.GrabBlock()
    place_block_place = actions.PlaceOn()

    handover = actions.HandOver()
    move_to = actions.PlacePosition()

    place_pos = actions.PlacePosition()

    grab_bottle = actions.GrabBottle()
    open_bottle = actions.OpenBottle()

    # TowerPlanner.get_goal_controller(self.)

    C.view()


    lfinger1 = C.getFrame("L_panda_finger_joint2")
    rfingerjoin = C.getFrame("L_panda_finger_joint1")

    # for t in range(10):
    #     lfinger1.setRelativePosition((t / 100, 0, 0))
    #     rfingerjoin.setRelativePosition((t / 100, 0, 0))
    #     time.sleep(0.1)

    eqPrecision = 1e-2

    tau = 0.01

    robust_plan = []
    # robust_plan.extend(grab_stick.get_grounded_control_set(C, ["R_gripper", "stick"]))
    # robust_plan.extend(pull_block.get_grounded_control_set(C, ["R_gripper", "b1", "stick"]))

    # robust_plan.extend(grab_block.get_grounded_control_set(C, ["R_gripper", "b1"]))
    robust_plan.extend(grab_bottle.get_grounded_control_set(C, ["R_gripper", "bottle"]))
    robust_plan.extend(open_bottle.get_grounded_control_set(C, ["R_gripper", "L_gripper", "bottle"]))
    # robust_plan.extend(place_block_place.get_grounded_control_set(C, ["R_gripper", "b1", "b2"]))
    # robust_plan.extend(handover.get_grounded_control_set(C, ["R_gripper", "L_gripper", "b1"]))
    # robust_plan.extend(place_pos.get_grounded_control_set(C, ["R_gripper", "b2"]))

    for name, a in robust_plan:
        pass
        # a.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e1]), ry.OT.ineq)

    bot = pybot.BotOp(C, False)
    q_start = C.getJointState()
    q_shaped = q_start.reshape(1, 14)
    bot.move(q_shaped, [2])

    ref_tau = 0.05

    while bot.getTimeToEnd() > 0:
        bot.step(C, .1)
        time.sleep(.1)

    do_once = True

    for t in range(0, 10000):

        t_0 = time.time()

        q_real = C.getJointState()
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, (name, c) in enumerate(robust_plan[::-1]):
            if c.canBeInitiated(ctrl, eqPrecision):
                print("Inting controller: ", name)
                ctrl.set(c)

                # if t > 500 and do_once:
                #     C.attach("world", "cap")
                #     do_once = False
                break
            # update simulation/ make a step
        ctrl.update(q_real, [], C)
        q = ctrl.solve(C)

        cap = C.frame("cap").info()
        bottle = C.frame("bottle").info()
        a = 1


        # print(q.reshape(1, 7))
        # C.setJointState(q)
        # C.computeCollisions()
        # coll = C.getCollisions(0)

        bot.moveLeap(q, 2)
        bot.step(C, 0)

        t_1 = time.time()
        t_delta = t_1 - t_0
        if t_delta > ref_tau:
            time.sleep(t_delta - ref_tau)
