import libry as ry
import libpybot as pybot
import time
import actions
from util.setup_env import setup_tower_env, setup_pick_and_place_env, setup_hand_over_env
from robustness import get_robust_chain
from planners import TowerPlanner


if __name__ == '__main__':

    C, block_names = setup_tower_env(2)

    grab_stick = actions.GrabStick()
    pull_block = actions.PullBlockStick()

    grab_block = actions.GrabBlock()
    place_block_place = actions.PlaceOn()

    handover = actions.HandOver()
    move_to = actions.PlacePosition()

    place_pos = actions.PlacePosition()

    # TowerPlanner.get_goal_controller(self.)

    C.view()

    eqPrecision = 1e-2

    tau = 0.01

    robust_plan = []
    # robust_plan.extend(grab_stick.get_grounded_control_set(C, ["R_gripper", "stick"]))
    # robust_plan.extend(pull_block.get_grounded_control_set(C, ["R_gripper", "b1", "stick"]))

    robust_plan.extend(grab_block.get_grounded_control_set(C, ["R_gripper", "b1"]))
    robust_plan.extend(place_block_place.get_grounded_control_set(C, ["R_gripper", "b1", "b2"]))
    #robust_plan.extend(handover.get_grounded_control_set(C, ["R_gripper", "L_gripper", "b1"]))
    #robust_plan.extend(place_pos.get_grounded_control_set(C, ["R_gripper", "b2"]))


    for name, a in robust_plan:
        pass
        #a.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e2]), ry.OT.ineq)

    block = C.getFrame("b1")
    print(block.info())

    bot = pybot.BotOp(C, True)
    q_start = C.getJointState()

    bot.move(q_start[:7].reshape(1, 7), [2])

    ref_tau = 0.05

    robust_plan = robust_plan[:-1]

    while bot.getTimeToEnd() > 0:
        bot.step(C, .1)
        time.sleep(.1)

    for t in range(0, 10000):

        t_0 = time.time()

        q_real = C.getJointState()
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, (name, c) in enumerate(robust_plan[::-1]):
            if c.canBeInitiated(ctrl, eqPrecision):
                print("Inting controller: ", name)
                ctrl.set(c)
                break
            # update simulation/ make a step
        ctrl.update(q_real, [], C)
        q = ctrl.solve(C)
        #print(q.reshape(1, 7))
        # C.setJointState(q)
        #C.computeCollisions()
        #coll = C.getCollisions(0)

        bot.moveLeap(q, 2)
        bot.step(C, 0)

        t_1 = time.time()
        t_delta = t_1 - t_0
        if t_delta > ref_tau:
            time.sleep(t_delta-ref_tau)

