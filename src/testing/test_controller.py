import libry as ry
import libpybot as pybot
import time
import actions
from util import setup_env, constants
from tracking import Tracker

if __name__ == '__main__':

    C, scene_objects = setup_env.setup_hand_over_env()

    grab_stick = actions.GrabStick()
    pull_block = actions.PullBlockToGoal()

    grab_block = actions.GrabBlock()
    place_block_place = actions.PlaceOn()

    handover = actions.HandOver()
    move_to = actions.PlaceGoal()

    place_pos = actions.PlaceGoal()

    grab_bottle = actions.GrabBottle()
    open_bottle = actions.OpenBottle()

    # TowerPlanner.get_goal_controller(self.)

    use_real_robot = False

    C.view()

    print([x for x in C.getFrameNames() if "gripper" in x])

    # time.sleep(10)

    # lfinger1 = C.getFrame("L_panda_finger_joint2")
    # rfingerjoin = C.getFrame("L_panda_finger_joint1")

    # for t in range(10):
    #     lfinger1.setRelativePosition((t / 100, 0, 0))
    #     rfingerjoin.setRelativePosition((t / 100, 0, 0))
    #     time.sleep(0.1)

    eqPrecision = 1e-2

    tau = 0.1

    gripper_take, gripper_give = "l_gripper", "r_gripper"

    if False:
        gripper_take, gripper_give = gripper_give, gripper_take

    robust_plan = []




    # test block pull
    # constants.goal_block_pos = constants.goal_stick_pull_block_pos
    # robust_plan.extend(grab_stick.get_grounded_control_set(C, ["r_gripper", "stick"]))
    # robust_plan.extend(pull_block.get_grounded_control_set(C, ["r_gripper", "b1", "stick"]))

    # robust_plan.extend(grab_block.get_grounded_control_set(C, ["R_gripper", "b1"]))
    # robust_plan.extend(grab_bottle.get_grounded_control_set(C, ["R_gripper", "bottle"]))
    # robust_plan.extend(open_bottle.get_grounded_control_set(C, ["R_gripper", "L_gripper", "bottle"]))
    # robust_plan.extend(place_block_place.get_grounded_control_set(C, ["R_gripper", "b1", "b2"]))

    # Test handover
    constants.goal_block_pos = constants.goal_handover_block_pos
    robust_plan.extend(grab_block.get_grounded_control_set(C, [gripper_give, "b1"]))
    robust_plan.extend(handover.get_grounded_control_set(C, [gripper_give, gripper_take, "b1"]))
    robust_plan.extend(place_pos.get_grounded_control_set(C, [gripper_take, "b1"]))



    for name, a in robust_plan:
        pass
        # a.addObjective(C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e-1]), ry.OT.ineq)

    bot = pybot.BotOp(C, use_real_robot, "BOTH", "ROBOTIQ")
    # q_start = C.getJointState()
    # q_start = q_start.reshape(1, 14)
    # bot.move(q_start, [2])

    ref_tau = 0.05

    bot.gripperOpen(1, 1, 1)

    while not bot.gripperDone(1):
        time.sleep(0.1)

    # bot.gripperOpen("RIGHT", 1.0, 0.2)

    do_once = True
    q_old = C.getJointState()

    if use_real_robot:
        tracker = Tracker(C, [x for y in scene_objects.values() for x in y], 1)

    for t in range(0, 10000):

        if use_real_robot:
            tracker.update(t)

        q_real = C.getJointState()
        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, 0.1, 2)

        for i, (name, c) in enumerate(robust_plan[::-1]):
            if c.canBeInitiated(ctrl, eqPrecision):
                ctrl.set(c)
                print(name)
                if name == "GrabStick_grab":
                    bot.gripperClose(1, 1, 0.01, 0.1)

                    while not bot.gripperDone(1):
                        time.sleep(0.1)

                if name == "PullBlockToGoal_attach_handle_stick":
                    print("was attached!")

                # if t > 500 and do_once:
                #     C.attach("world", "cap")
                #     do_once = False
                break
            # update simulation/ make a step
        q_dot = q_real - q_old
        ctrl.update(q_real, q_dot, C)
        q = ctrl.solve(C)

        # cap = C.frame("cap").info()
        # bottle = C.frame("bottle").info()
        # a = 1

        # print(q.reshape(1, 7))
        # C.setJointState(q)
        # C.computeCollisions()
        # coll = C.getCollisions(0)

        bot.moveLeap(q, 2)
        bot.step(C, 0.1)

        # t_1 = time.time()
        # t_delta = t_1 - t_0
        # if t_delta > ref_tau:
        #     time.sleep(t_delta - ref_tau)
