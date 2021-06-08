from scipy.spatial.transform.rotation import Rotation
import libry as ry
import networkx as nx
import numpy as np

import util.constants as dt
import actions
from feasibility import check_switch_chain_feasibility
from robustness import get_robust_set_of_chains
from util.visualize_search import draw_search_graph


class RLGS:

    def __init__(self, C,
                 use_robust=True,
                 use_feasy=True,
                 use_multi=True,
                 verbose=False,
                 show_plts=False):

        # what the robot knows
        self.C = ry.Config()
        self.C.copy(C)
        self.use_feasy = use_feasy
        self.use_robust = use_robust
        self.use_multi = use_multi
        self.verbose = verbose

        # store plan and action tree here
        self.scene_objects = {}
        self.robust_set_of_chains = []
        self.active_robust_reverse_plan = []
        self.goal_controller = ry.CtrlSet()

        # stuff for execution
        self.is_done = False
        self.feasy_check_rate = 50  # every 50 steps check for feasibility
        self.C.view()
        self.gripper_action = None

    def setup(self, action_list, planner, scene_objects):

        # put all objects into one dictionary with types
        self.scene_objects = scene_objects

        plan, goal, state_plan, action_tree = planner.get_plan(action_list, self.scene_objects)

        self.goal_controller = planner.get_goal_controller(self.C)

        # if there is a plan, print it out, otherwise leave
        if plan:
            if self.verbose:
                print("Found the following plan:")
                for action in plan:
                    print(action)
        else:
            print("No plan found!")
            print("Aborting")
            return

        # setup controlsets

        name2con = {x.name: x for x in action_list}
        grounded_actions = nx.get_edge_attributes(action_tree, "action")
        grounded_ctrlsets = dict()

        for edge in action_tree.edges():
            grounded_action = grounded_actions[edge]
            relevant_frames = grounded_action.sig[1:]
            controller = name2con[grounded_action.sig[0]]  # the actual controller
            grounded_ctrlsets[edge] = controller.get_grounded_control_set(self.C, relevant_frames)

        # each edge (action) gets and ctrlset
        nx.set_edge_attributes(action_tree, grounded_ctrlsets, "ctrlset")

        # draw the action tree
        draw_search_graph(plan, state_plan, action_tree)

        # get robust tree/ set of chains
        self.robust_set_of_chains = get_robust_set_of_chains(self.C, action_tree, state_plan, self.goal_controller,
                                                             True)

        # first plan we want to execute
        first_plan = self.robust_set_of_chains[0]

        # check if plan is feasible in current config
        is_feasible, komo_feasy = check_switch_chain_feasibility(self.C, first_plan, self.goal_controller,
                                                                 self.scene_objects, verbose=self.verbose)
        self.active_robust_reverse_plan = first_plan[::-1]

        tau = 0.01
        for name, x in nx.get_edge_attributes(action_tree, "implicit_ctrlsets").items():
            for y in x:
                pass
                y.add_qControlObjective(2, 1e-5 * np.sqrt(tau),
                                        self.C)  # TODO this will make some actions unfeasible (PlaceSide)
                y.add_qControlObjective(1, 1e-3 * np.sqrt(tau), self.C)
                # TODO enabling contact will run into local minima, solved with MPC (Leap Controller from Marc)
                # TODO, throws errors! RuntimeError: /home/jason/git/thesis_2020/rai/rai/Geo/pairCollision.cpp:PairCollision:78(-2) CHECK_GE failed: 'rai::sign(distance) * scalarProduct(normal, p1-p2)'=-nan '-1e-10'=-1e-10 --
                y.addObjective(self.C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e1]), ry.OT.ineq)

        if not is_feasible:
            print("Plan is not feasible in current Scene!")
            print("Aborting")
            return False

        else:
            return True

    def is_goal_fulfilled(self):
        return self.goal_controller.canBeInitiated(self.C)

    def get_gripper_action(self):

        return self.gripper_action

    def step(self, t, tau):

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, tau, 2)
        is_any_controller_feasible = False
        is_current_plan_feasible = True
        is_any_plan_feasible = True
        self.gripper_action = None

        # iterate over each controller, check which can be started first
        for i, (name, c) in enumerate(self.active_robust_reverse_plan):
            if c.canBeInitiated(self.C):
                ctrl.set(c)
                is_any_controller_feasible = True

                # check if we grabbing something
                for ctrlCommand in c.getSymbolicCommands():
                    if not ctrlCommand.isCondition():
                        self.gripper_action = ctrlCommand

                # check feasibility of chain
                if not t % self.feasy_check_rate:
                    # check rest of chain for feasibility
                    residual_plan = self.active_robust_reverse_plan[i::-1]
                    is_plan_feasible, _ = check_switch_chain_feasibility(self.C, residual_plan, self.goal_controller,
                                                                         self.scene_objects, verbose=False)
                if self.verbose:
                    print(f"Initiating: {name}")
                # leave loop, we have the controller
                break
            else:
                if self.verbose or True:
                    print(f"Cannot be initiated: {name}")

        # if current plan is not feasible, check other plans
        if not is_any_controller_feasible or not is_current_plan_feasible:
            print("No controller can be initiated!")

            # lets switch the plan
            for plan in self.robust_set_of_chains[1:]:
                is_initiated = False
                is_feasible = False
                for i, (name, c) in enumerate(plan[::-1]):
                    if c.canBeInitiated(self.C):
                        new_plan = plan
                        is_feasible, komo_feasy = check_switch_chain_feasibility(self.C, new_plan, self.goal_controller,
                                                                                 self.scene_objects, verbose=False)
                        is_initiated = True
                        break
                    elif self.verbose:
                        print(f"{name} cannot be initiated!")
                if is_initiated and is_feasible:
                    print("new plan found!")
                    print(plan)
                    self.active_robust_reverse_plan = plan[::-1]
                    break

        ctrl.update(self.C)
        q = ctrl.solve(self.C)
        self.C.setJointState(q)

        # TODO add info is feasibility failed
        return q

    def cheat_update_obj(self, object_infos):
        for obj_name, obj_info in object_infos.items():

            obj = self.C.frame(obj_name)
            obj.setPosition(obj_info['pos'])
            shape = obj_info['shape']

            # TODO: this rotation doesn't seem to work...
            rot = Rotation.from_matrix(obj_info['rot_max'])
            # obj.setQuaternion(rot.as_quat())
            return

    def set_gripper_width(self, gripper):

        self.C.setJointState()