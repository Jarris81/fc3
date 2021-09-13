from scipy.spatial.transform.rotation import Rotation
import libry as ry
import networkx as nx
import numpy as np
import time

from feasibility import check_switch_chain_feasibility
from robustness import get_robust_set_of_chains
from util.visualize_search import draw_search_graph
import libpybot as pybot

from tracking import Tracker


class SimpleSystem:
    """
    Simple Linear/Sequential Execution Model and Template for others
    """

    def __init__(self, C,
                 verbose=False,
                 show_plts=False,
                 use_real_robot=False,
                 use_tracking=False):
        # what the robot knows
        self.C = C
        self.verbose = verbose
        self.use_real_robot = use_real_robot
        self.use_tracking = use_tracking

        # action tree received from planner
        self.action_tree = None
        self._use_single_path = True

        # store plan and action tree here
        self.scene_objects = {}
        self.active_plan = []
        self.goal_controller = ry.CtrlSet()
        self.current_active_controller_index = 0

        # joint values
        self.q = None
        self.q_old = None
        self.tau = 0.01

        # precision needed to initiate a controller
        self.eqPrecision = 1e-2

        # set logger function
        self.log = self.log_default

        # robot operator
        self.botop = None

        # tracking system (OptiTrack)
        self.tracker = None

        # gripper to robot index
        self.gripper2index = {"l_gripper": 0, "r_gripper": 1}

    def log_default(self, msg):
        if self.verbose:
            print(msg)

    def init_system(self, action_list, planner, scene_objects) -> bool:

        self.q = self.C.getJointState()
        self.q_old = self.q

        # put all objects into one dictionary with types
        self.scene_objects = scene_objects

        self.action_tree = planner.get_tree(action_list, self.scene_objects, forward=self._use_single_path)

        self.goal_controller = planner.get_goal_controller(self.C)

        # setup control sets
        name2con = {x.name: x for x in action_list}
        grounded_actions = nx.get_edge_attributes(self.action_tree, "action")
        grounded_ctrlsets = dict()

        for edge in self.action_tree.edges():
            grounded_action = grounded_actions[edge]
            relevant_frames = grounded_action.sig[1:]
            controller = name2con[grounded_action.sig[0]]  # the actual controller
            grounded_ctrlsets[edge] = controller.get_grounded_control_set(self.C, relevant_frames)

            self.active_plan.extend(grounded_ctrlsets[edge])

        # each edge (action) gets and ctrlset
        nx.set_edge_attributes(self.action_tree, grounded_ctrlsets, "ctrlset")

        draw_search_graph(self.action_tree, "simple_plan.png")

        # convert all to a plan

        return True

    def move_home(self):

        self.botop.home(self.C)

    def is_goal_fulfilled(self):
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        return self.goal_controller.canBeInitiated(ctrl, self.eqPrecision)

    def setup(self):
        self.botop = pybot.BotOp(self.C, self.use_real_robot, "both", "ROBOTIQ")

        self.tracker = Tracker(self.C,
                               [x for y in self.scene_objects.values() for x in y],
                               1) \
            if self.use_tracking else None

    def run(self):


        t_start = self.botop.get_t()
        while not self._is_done():
            if self.tracker: self.tracker.update(self.botop.get_t())
            self._step(self.botop.get_t())

        t_total = self.botop.get_t() - t_start

        self.log(f"Task took {t_total}s.")

    def _step(self, t):

        q_real = self.C.getJointState()
        q_target = q_real

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)

        # get the current controller
        curr_name, curr_c = self.active_plan[self.current_active_controller_index]

        # chek if not last controller
        if not self.current_active_controller_index == len(self.active_plan) - 1:
            next_name, next_c = self.active_plan[self.current_active_controller_index + 1]
            # if next can be executed, run that
            if next_c.canBeInitiated(ctrl, self.eqPrecision):
                print("Moving to next controller")
                self.current_active_controller_index += 1

        if curr_c.canBeInitiated(ctrl, self.eqPrecision):
            self.log(f"Initiating: {curr_name}")
            ctrl.set(curr_c)

            q_dot = self.q - self.q_old
            ctrl.update(q_real, q_dot, self.C)
            q_target = ctrl.solve(self.C)

            # Handle symbolic commands here
            for ctrlCommand in curr_c.getSymbolicCommands():
                self._handle_symbolic_commands(ctrlCommand)

        self.q = q_real
        self.q_old = self.q_old

        self.botop.moveLeap(q_target, 2)
        self.botop.step(self.C, 0.05)

    def _move_up_safely(self, gripper):

        gripper_up_pos = self.C.frame(gripper).getPosition()
        gripper_up_pos[2] += 0.1  # move up 10 cm

        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        transient_step = 0.1

        move_up = ry.CtrlSet()
        tau = 0.01
        move_up.add_qControlObjective(2, 1e-3 * np.sqrt(tau),
                                      self.C)  # TODO this will make some actions unfeasible (PlaceSide)
        move_up.add_qControlObjective(1, 1e-3 * np.sqrt(tau), self.C)
        move_up.addObjective(
            self.C.feature(ry.FS.position, [gripper], [1e2], gripper_up_pos),
            ry.OT.sos, transient_step / 5)

        move_up.addObjective(
            self.C.feature(ry.FS.vectorZDiff, ["world", gripper], [1e1]),
            ry.OT.sos, transient_step / 5)

        ctrl.set(move_up)

        while not np.allclose(self.C.frame(gripper).getPosition(), gripper_up_pos):
            q_real = self.C.getJointState()
            q_dot = self.q - self.q_old

            ctrl.update(q_real, q_dot, self.C)
            q_target = ctrl.solve(self.C)

            self.botop.moveLeap(q_target, 2)
            self.botop.step(self.C, 0.05)

            self.q = q_real
            self.q_old = self.q_old

    def _is_done(self):

        return self.is_goal_fulfilled()

    def _handle_symbolic_commands(self, ctrlCommand):
        """
        Define what should happen if a symbolic command is executed
        """
        if ctrlCommand.isCondition():
            return  # only do stuff if it is not a condition

        elif ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
            self.botop.hold(True, False)
            self.botop.gripperClose(
                self.gripper2index[ctrlCommand.getFrameNames()[0]], 0.5, 0.01, 0.1)
            while not self.botop.gripperDone(1):
                time.sleep(0.1)

        elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER:
            self.botop.hold(True, True)
            self.botop.gripperOpen(
                self.gripper2index[ctrlCommand.getFrameNames()[0]], 1, 0.1)
            while not self.botop.gripperDone(1):
                time.sleep(0.1)
            # Cheating, but for now always move up after placing something
            self._move_up_safely(ctrlCommand.getFrameNames()[0])


class RLGS(SimpleSystem):
    """
    Model Execution Class for this Thesis
    """

    def __init__(self, C,
                 use_real_robot=False,
                 use_feasy=True,
                 verbose=False,
                 show_plts=False):

        # what the robot knows
        super().__init__(C, use_real_robot=use_real_robot,
                         show_plts=show_plts,
                         verbose=verbose)

        # Robust system
        self.use_feasy = use_feasy
        # we want multiple paths
        self._use_single_path = False

        # store plan and action tree here
        self.scene_objects = {}
        self.robust_set_of_chains = []
        self.active_robust_reverse_plan = []
        self.goal_controller = ry.CtrlSet()
        self.no_plan_feasible = False

        # stuff for execution
        self.feasy_check_rate = 20  # every 50 steps check for feasibility

    def init_system(self, action_list, planner, scene_objects):

        super().init_system(action_list, planner, scene_objects)

        # del self.active_plan

        # get robust tree/ set of chains
        self.robust_set_of_chains = get_robust_set_of_chains(self.C, self.action_tree, self.goal_controller,
                                                             verbose=self.verbose)
        # first plan we want to execute
        self.active_robust_reverse_plan = self.get_feasible_reverse_plan(ry.CtrlSolver(self.C, 0.1, 2))

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

    def is_no_plan_feasible(self):
        return self.no_plan_feasible

    def _step(self, t):

        q_real = self.C.getJointState()
        q_target = q_real

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        is_any_controller_feasible = False
        is_current_plan_feasible = False

        self.current_active_controller_index = 0

        # iterate over each controller, check which can be started first
        for i, (edge, name, c) in enumerate(self.active_robust_reverse_plan):

            if c.canBeInitiated(ctrl, self.eqPrecision):
                self.log(f"Initiating: {name}")
                ctrl.set(c)
                is_any_controller_feasible = True
                is_current_plan_feasible = True
                self.current_active_controller_index = i

                q_dot = self.q - self.q_old
                ctrl.update(q_real, q_dot, self.C)
                q_target = ctrl.solve(self.C)

                self.q = q_real
                self.q_old = self.q_old

                self.botop.moveLeap(q_target, 2)
                self.botop.step(self.C, 0.01)

                # Handle symbolic commands here
                for ctrlCommand in c.getSymbolicCommands():
                    self._handle_symbolic_commands(ctrlCommand)

                # leave loop, we have the controller
                break
            else:
                pass
                # self.log(f"Cannot be initiated: {name}")

        # check feasibility of chain
        if self.use_feasy and not t % self.feasy_check_rate and len(self.active_robust_reverse_plan):
            # check rest of controller chain for feasibility
            residual_plan = self.active_robust_reverse_plan[self.current_active_controller_index::-1]
            is_current_plan_feasible, _ = check_switch_chain_feasibility(self.C, residual_plan,
                                                                         self.goal_controller,
                                                                         self.scene_objects, verbose=self.verbose)

        # if current plan is not feasible, check other plans
        if self.use_feasy and (
                not is_any_controller_feasible or not is_current_plan_feasible) and not t % self.feasy_check_rate:
            self.log("No controller can be initiated or current plan is not feasible!")
            self.active_robust_reverse_plan = self.get_feasible_reverse_plan(ctrl)


    def get_feasible_reverse_plan(self, ctrl):
        # find a new feasible plan
        for plan in self.robust_set_of_chains:
            for i, (edge, name, c) in enumerate(plan[::-1]):
                if c.canBeInitiated(ctrl, self.eqPrecision):
                    self.log(f"{edge} CAN be initiated!")
                    residual_plan = plan[-i - 1:]
                    is_feasible, komo_feasy = check_switch_chain_feasibility(self.C, residual_plan,
                                                                             self.goal_controller,
                                                                             self.scene_objects,
                                                                             verbose=self.verbose)
                    if is_feasible:
                        if self.verbose:
                            print("new plan found!")
                            print(residual_plan)
                        return residual_plan[::-1]
                    else:
                        self.log(f"Plan {[x[1] for x in residual_plan]} is not feasible!")
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

    def log_default(self, msg):
        if self.verbose:
            print(msg)

    def set_log_function(self, log_function):
        self.log = log_function

    def _is_done(self):

        return self.is_goal_fulfilled() or self.is_no_plan_feasible()


class RLDSClone(RLGS):

    def __init__(self, C, verbose=False, use_real_robot=False):
        super().__init__(C, use_feasy=False, verbose=verbose, use_real_robot=use_real_robot)
