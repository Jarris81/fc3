from scipy.spatial.transform.rotation import Rotation
import libry as ry
import networkx as nx
import numpy as np

from feasibility import check_switch_chain_feasibility
from robustness import get_robust_set_of_chains
from util.visualize_search import draw_search_graph


class SimpleSystem:

    def __init__(self, C,
                 verbose=False,
                 show_plts=False):
        # what the robot knows
        self.C = C
        self.verbose = verbose

        # action tree received from planner
        self.action_tree = None

        # store plan and action tree hereRotation
        self.scene_objects = {}
        self.robust_set_of_chains = []
        self.active_robust_reverse_plan = []
        self.goal_controller = ry.CtrlSet()
        self.no_plan_feasible = False

        # stuff for execution
        self.is_done = False
        self.feasy_check_rate = 20  # every 50 steps check for feasibility
        self.gripper_action = None

        self.q = None
        self.q_old = None

        # precision needed to initiate a controller
        self.eqPrecision = 1e-2

        # home controller, used when robot is done with plan
        self.home_controller = ry.CtrlSet()
        self.q_home = C.getJointState()
        self.home_controller.addObjective(
            C.feature(ry.FS.qItself, [], [1e2], self.q_home),
            ry.OT.sos, 0.05
        )
        self.home_controller.add_qControlObjective(2, 1e-5 * np.sqrt(0.01),
                                                   self.C)  # TODO this will make some actions unfeasible (PlaceSide)
        self.home_controller.add_qControlObjective(1, 1e-3 * np.sqrt(0.01), self.C)

        self.log = self.log_default

    def log_default(self, msg):
        if self.verbose:
            print(msg)

    def setup(self, action_list, planner, scene_objects):

        self.q = self.C.getJointState()
        self.q_old = self.q

        # put all objects into one dictionary with types
        self.scene_objects = scene_objects

        self.action_tree = planner.get_tree(action_list, self.scene_objects, forward=False)

        self.goal_controller = planner.get_goal_controller(self.C)

        # if there is a plan, print it out, otherwise leave
        # if plan:
        #     if self.verbose:
        #         print("Found the following plan:")
        #         for action in plan:
        #             print(action)
        # else:
        #     print("No plan found!")
        #     print("Aborting")
        #     return

        # setup control sets
        name2con = {x.name: x for x in action_list}
        grounded_actions = nx.get_edge_attributes(self.action_tree, "action")
        grounded_ctrlsets = dict()

        for edge in self.action_tree.edges():
            grounded_action = grounded_actions[edge]
            relevant_frames = grounded_action.sig[1:]
            controller = name2con[grounded_action.sig[0]]  # the actual controller
            grounded_ctrlsets[edge] = controller.get_grounded_control_set(self.C, relevant_frames)

        # each edge (action) gets and ctrlset
        nx.set_edge_attributes(self.action_tree, grounded_ctrlsets, "ctrlset")

        # draw the action tree
        # draw_search_graph(plan, state_plan, self.action_tree)

        #
        # self.controllers =

    def is_home(self):
        is_home = np.allclose(self.C.getJointState(), self.q_home, self.eqPrecision, self.eqPrecision)
        return is_home


class RLGS(SimpleSystem):

    def __init__(self, C, use_robust=True, use_feasy=True, use_multi=True, verbose=False, show_plts=False):

        # what the robot knows
        super().__init__(C, verbose, show_plts)
        self.C = C
        self.use_feasy = use_feasy
        self.use_robust = use_robust
        self.use_multi = use_multi
        self.verbose = verbose

        # store plan and action tree here
        self.scene_objects = {}
        self.robust_set_of_chains = []
        self.active_robust_reverse_plan = []
        self.goal_controller = ry.CtrlSet()
        self.no_plan_feasible = False

        # stuff for execution
        self.feasy_check_rate = 20  # every 50 steps check for feasibility

    def setup(self, action_list, planner, scene_objects):

        super().setup(action_list, planner, scene_objects)

        # get robust tree/ set of chains
        self.robust_set_of_chains = get_robust_set_of_chains(self.C, self.action_tree, self.goal_controller,
                                                             verbose=self.verbose)
        # first plan we want to execute
        self.active_robust_reverse_plan = self.get_feasible_reverse_plan(ry.CtrlSolver(self.C, 0.1, 2),
                                                                         verbose=self.verbose)


        tau = 0.01
        for name, x in nx.get_edge_attributes(self.action_tree, "implicit_ctrlsets").items():
            for name, y in x:
                pass
                y.add_qControlObjective(2, 1e-3 * np.sqrt(tau),
                                        self.C)  # TODO this will make some actions unfeasible (PlaceSide)
                y.add_qControlObjective(1, 1e-3 * np.sqrt(tau), self.C)
                # TODO enabling contact will run into local minima, solved with MPC (Leap Controller from Marc)

                # y.addObjective(self.C.feature(ry.FS.accumulatedCollisions, ["ALL"], [1e0]), ry.OT.ineq)

        return True

    def is_goal_fulfilled(self):
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        return self.goal_controller.canBeInitiated(ctrl, self.eqPrecision)

    def is_no_plan_feasible(self):
        return self.no_plan_feasible

    def get_gripper_action(self):

        return self.gripper_action

    def step(self, t, tau):

        q_real = self.C.getJointState()

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        is_any_controller_feasible = False
        is_current_plan_feasible = False
        self.gripper_action = None

        current_controller_index = 0

        # iterate over each controller, check which can be started first
        for i, (edge, name, c) in enumerate(self.active_robust_reverse_plan):
            if c.canBeInitiated(ctrl, self.eqPrecision):
                self.log(f"Initiating: {name}")
                ctrl.set(c)
                is_any_controller_feasible = True
                is_current_plan_feasible = True
                current_controller_index = i

                for ctrlCommand in c.getSymbolicCommands():
                    if ctrlCommand.isCondition():
                        continue
                    elif ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                        self.gripper_action = True
                        break
                    elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
                        self.gripper_action = False

                # leave loop, we have the controller
                break
            else:
                pass
                # self.log(f"Cannot be initiated: {name}")
        # check feasibility of chain
        if not t % self.feasy_check_rate and len(self.active_robust_reverse_plan):
            # check rest of chain for feasibility
            residual_plan = self.active_robust_reverse_plan[current_controller_index::-1]
            is_current_plan_feasible, _ = check_switch_chain_feasibility(self.C, residual_plan,
                                                                         self.goal_controller,
                                                                         self.scene_objects, verbose=False)

        # # if current plan is not feasible, check other plans
        if (not is_any_controller_feasible or not is_current_plan_feasible) and not t % self.feasy_check_rate:
            print("No controller can be initiated or current plan is not feasible!")
            self.active_robust_reverse_plan = self.get_feasible_reverse_plan(ctrl, False)

        q_dot = self.q - self.q_old

        ctrl.update(q_real, q_dot, self.C)
        q = ctrl.solve(self.C)

        # TODO add info is feasibility failed
        self.q = q_real
        self.q_old = self.q_old
        return q, self.gripper_action

    def get_feasible_reverse_plan(self, ctrl, verbose=False):
        # find a new feasible plan
        for plan in self.robust_set_of_chains:
            for i, (edge, name, c) in enumerate(plan[::-1]):
                if c.canBeInitiated(ctrl, self.eqPrecision):
                    self.log(f"{edge} CAN be initiated!")
                    residual_plan = plan[-i - 1:]
                    is_feasible, komo_feasy = check_switch_chain_feasibility(self.C, residual_plan,
                                                                             self.goal_controller,
                                                                             self.scene_objects,
                                                                             verbose=verbose)
                    if is_feasible:
                        if self.verbose:
                            print("new plan found!")
                            print(residual_plan)
                        return residual_plan[::-1]
                    else:
                        self.log(f"Plan {[x[1] for x in  residual_plan]} is not feasible!")
                elif self.verbose:
                    print(f"{edge} cannot be initiated!")

        # if no plan is feasible, return false
        return []

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

    def log_default(self, msg):
        if self.verbose:
            print(msg)

    def set_log_function(self, log_function):
        self.log = log_function



