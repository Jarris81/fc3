import libry as ry
import time
from util.setup_env import setup_tower_env
from actions import GrabBlock, PlaceSide, PlaceOn
from predicates import BlockOnBlock

from planners import get_tower_goal_controller

from robustness import get_robust_system_2

if __name__ == '__main__':

    R, C, block_names = setup_tower_env(3)

    grab_block = GrabBlock()
    place_block = PlaceOn()
    place_side = PlaceSide()

    gripper_name = "R_gripper"

    all_frames = set(block_names + [gripper_name])

    controllers = []

    controllers.extend(grab_block.get_grounded_control_set(C, (gripper_name, "b1"), all_frames))
    controllers.extend(place_block.get_grounded_control_set(C, (gripper_name, "b1", "b2"), all_frames))

    controller_tuples = []

    for i, controller in enumerate(controllers):
        controller_tuples.append((f"{i}", controller))

    b_on_b = BlockOnBlock("B", "B_2")
    b_on_b.ground_predicate(B="b1", B_2="b2")


    goal_controller = get_tower_goal_controller(C, [b_on_b.get_grounded_predicate()])

    robust_plan = get_robust_system_2(C, controller_tuples, goal_controller, False)

    C.view()
    tau = .01

    verbose = True
    all_controllers_unfeasible = True

    interference_counter = 0
    # say where we want intereference
    has_interfered = {1: False, 3: False}



    robust_plan.append()

    # simulation loop
    for t in range(0, 10000):

        if goal_controller.canBeInitiated(C):
            is_done = True
            break

        # reset, to check if at least one controller can be initiated
        all_controllers_unfeasible = False

        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)

        for i, (name, c) in enumerate(robust_plan):
            if c.canBeInitiated(C):
                ctrl.set(c)
                all_controllers_unfeasible = False
                if i in has_interfered and not has_interfered[i]:  # 3 works, 1 doesnt
                    interference_counter += 1
                    if interference_counter == 50:
                        block = C.frame("b2")
                        block.setPosition(ori)
                        has_interfered[i] = True
                        interference_counter = 0

                if verbose:
                    print(f"Initiating: {name}")
                break
            else:
                if verbose:
                    print(f"Cannot be initiated: {name}")

        if all_controllers_unfeasible and verbose:
            print("No controller can be initiated!")

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        coll = C.getCollisions(0)
        time.sleep(tau)

    #C.setJointState(q)
    #C.computeCollisions()

    # C.view()
    #
    # tau = 0.01
    #
    # time.sleep(1)
    #
    # komo = ry.KOMO()
    # komo.setModel(C, False)
    #
    # komo.setTiming(1, 10, 5., 2)
    # komo.add_qControlObjective([], 2)
    # komo.addSquaredQuaternionNorms([], 3.)
    #
    # # Gripper Center close as possible
    # komo.addObjective([1], ry.FS.positionDiff, ["R_gripperCenter", "b1"], ry.OT.sos, [1e1]*3)
    #
    # # gripper should z axis should align with block (top grasp)
    # komo.addObjective([1], ry.FS.vectorZDiff, ["R_gripper", "b1"], ry.OT.sos, [1e2])
    #
    # #
    # komo.addObjective([], ry.FS.distance, ["R_gripper", "b1"], ry.OT.ineq, [1e1])
    #
    # komo.optimize()
    #
    # komo.view(False, "result")
    #
    # time.sleep(5)
    #
    # komo.view_play(.2, False)
    #
    # time.sleep(5)
    # report = komo.getReport(True)



