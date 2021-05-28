
import libry as ry

import util.domain_tower as dt
from util.setup_env import setup_tower_env
from actions import GrabBlock, PlaceOn, PlaceSide
from testing.tower_planner import get_plan, get_goal_controller

class RLGS:

    def __init__(self, use_robust, use_feasy, use_multi, verbose=False):

        self.use_feasy = use_feasy
        self.use_robust = use_robust
        self.use_multi = use_multi

        self.verbose = verbose

    def setup_tower(self):
        # get all actions needed to build a tower
        action_list = [
            GrabBlock(),
            PlaceOn(),
            PlaceSide()
        ]

        # setup config and get frame names
        _, C, block_names = setup_tower_env(3)  # 3 blocks in scene
        gripper_name = "R_gripper"

        # put all objects into one dictionary with types
        scene_objects = {
            dt.type_block: block_names,
            dt.type_gripper: (gripper_name,)
        }

        # get plan, goal, state plan and the search tree
        plan, goal, state_plan, G = get_plan(self.verbose, action_list, scene_objects)

    def step(self, t):

        # create a new solver everytime
        ctrl = ry.CtrlSolver(C, tau, 2)
        is_any_controllers_feasible = False

        # check if goal has been reached
        if goal_controller.canBeInitiated(C):
            is_done = True
            break

        # iterate over each controller, check which can be started first
        for i, (name, c) in enumerate(robust_plan):
            if c.canBeInitiated(C):
                ctrl.set(c)
                is_any_controllers_feasible = True
                # check feasibility of chain
                feasy_counter += 1
                if feasy_counter == feasy_check_rate:
                    feasy_counter = 0
                    # check rest of chain for feasibility
                    residual_plan = robust_plan[i::-1]
                    print(residual_plan)
                    is_plan_feasible, _ = check_switch_chain_feasibility(C, residual_plan, goal_controller,
                                                                         verbose=False)
                    print(is_plan_feasible)

                # TODO: move this outside
                if i == 1 and interference and not has_interfered:  # 3 works, 1 doesnt
                    interference_counter += 1
                    if interference_counter == 50:
                        block = C.frame("b2")
                        block.setPosition(original_position)
                        has_interfered = True
                if verbose:
                    print(f"Initiating: {name}")
                # leave loop, we have the controller
                break
            else:
                if verbose:
                    print(f"Cannot be initiated: {name}")

        # if no plan is feasible, check other plans
        if not is_any_controllers_feasible or not is_plan_feasible:
            print("No controller can be initiated!")
            # lets switch the plan

            for plan in robust_set_of_chains[1:]:
                is_initiated = False
                is_feasible = False
                for i, (name, c) in enumerate(plan[::-1]):
                    if c.canBeInitiated(C):
                        new_plan = plan
                        is_feasible, komo_feasy = check_switch_chain_feasibility(C, new_plan, goal_controller,
                                                                                 verbose=False)
                        is_initiated = True
                        break
                    else:
                        print(f"{name} cannot be initiated!")
                if is_initiated and is_feasible:
                    print("new plan found!")
                    print(plan)
                    robust_plan = plan[::-1]
                    break

        ctrl.update(C)
        q = ctrl.solve(C)
        C.setJointState(q)
        C.computeCollisions()
        # coll = C.getCollisions(0)
        time.sleep(tau)

    if is_done:
        print("Plan was finished!")
    else:
        print("Time ran out!")