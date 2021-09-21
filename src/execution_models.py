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

    def __init__(self, C, use_real_robot, use_tracking, verbose=False, show_plots=False):
        # what the robot knows
        self.C = C
        self.verbose = verbose
        self.use_real_robot = use_real_robot
        self.use_tracking = use_tracking
        self.show_plots = show_plots

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

        # leap step when using the leap controller
        self.leap_step = 0.05

        # tracking system (OptiTrack)
        self.tracker = None

        # gripper to robot index
        self.gripper2index = {"l_gripper": 0, "r_gripper": 1}

        # grasp lost
        self._grasp_lost = False

        # used for experiments
        self.time_is_up = False

    def log_default(self, msg):
        if self.verbose:
            print(msg)

    def init_system(self, action_list, planner, scene_objects) -> (bool, float):
        # put all objects into one dictionary with types
        self.scene_objects = scene_objects
        self.tracker = Tracker(self.C,
                               [x for y in self.scene_objects.values() for x in y],
                               1) \
            if self.use_tracking else None
        if self.tracker: self.tracker.update(0)

        t_start = time.time()

        self.q = self.C.getJointState()
        self.q_old = self.q

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

        return True, time.time() - t_start

    def move_home(self):

        self.botop.home(self.C)

    def is_goal_fulfilled(self):
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        return self.goal_controller.canBeInitiated(ctrl, self.eqPrecision)

    def setup(self):
        self.botop = pybot.BotOp(self.C, self.use_real_robot, "both", "ROBOTIQ")
        self.botop.gripperOpen(self.gripper2index["r_gripper"], 1, 1)
        while not self.botop.gripperDone(1):
            time.sleep(0.01)

    def run(self, run_interference, max_time=40):

        t_start = self.botop.get_t()

        while not self._is_done() and not self.time_is_up:

            if self.tracker:
                self.tracker.update(self.botop.get_t())
            if not self.use_real_robot:
                run_interference.do_interference(self.C, self.botop.get_t())

            self._step(self.botop.get_t())


            if self.botop.get_t() - t_start > max_time:
                self.time_is_up = True

        t_total = self.botop.get_t() - t_start

        self.log(f"Task took {t_total}s.")

        return t_total

    def _step(self, t):

        q_real = self.C.getJointState()
        q_target = q_real

        self._check_if_still_grasping()

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
        self.botop.step(self.C, self.leap_step)

    def _move_away_safely(self, gripper, block):

        safe_dist = 0.1

        transient_step = 0.05

        move_up = ry.CtrlSet()
        # move_up.add_qControlObjective(2, 1e-3 * np.sqrt(tau),
        #                               self.C)  # TODO this will make some actions unfeasible (PlaceSide)
        # move_up.add_qControlObjective(1, 1e-3 * np.sqrt(tau), self.C)
        move_up.addObjective(
            self.C.feature(ry.FS.positionRel, [block, gripper], [1e1], [0, 0, -safe_dist]),
            ry.OT.sos, transient_step)

        move_up.addObjective(
            self.C.feature(ry.FS.vectorZDiff, [gripper, block], [1e0]),
            ry.OT.sos, transient_step)

        at_place_controller = ry.CtrlSet()

        at_place_controller.addObjective(
            self.C.feature(ry.FS.positionRel, [block, gripper], [1e0], [0, 0, -safe_dist]),
            ry.OT.eq, -1)

        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        while not at_place_controller.canBeInitiated(ctrl, self.eqPrecision):
            q_real = self.C.getJointState()
            q_dot = self.q - self.q_old
            ctrl.set(move_up)
            ctrl.update(q_real, q_dot, self.C)
            q_target = ctrl.solve(self.C)

            self.botop.moveLeap(q_target, 2)
            self.botop.step(self.C, self.leap_step)

            ctrl = ry.CtrlSolver(self.C, 0.1, 2)

            self.q = q_real
            self.q_old = self.q_old
        print("done moving safely away")
        return

    def _is_done(self):

        return self.is_goal_fulfilled() or self.is_no_plan_feasible()

    def is_no_plan_feasible(self):
        return False

    def _check_if_still_grasping(self):
        if self._grasp_lost:
            self.C.attach("world", self._grasp_lost[1])
            gripper_index = self.gripper2index[self._grasp_lost[0]]
            self.botop.gripperOpen(gripper_index, 1, 1)

            while not self.botop.gripperDone(gripper_index):
                time.sleep(0.1)

            # Cheating, but for now always move up after placing something
            #self._move_away_safely(*self._grasp_lost)

            self._grasp_lost = False

    def _handle_symbolic_commands(self, ctrlCommand):
        """
        Define what should happen if a symbolic command is executed
        """

        if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER and not ctrlCommand.isCondition():
            if ctrlCommand.getFrameNames()[0] in self.gripper2index:
                gripper_index = self.gripper2index[ctrlCommand.getFrameNames()[0]]
                self.botop.gripperClose(gripper_index, 0.01, 0.03, 0.1)
                while not self.botop.gripperDone(gripper_index):
                    time.sleep(0.1)

                lost_grasp = self.botop.gripperGraspLost(gripper_index)
                if lost_grasp:
                    print(f"Lost grasp: {lost_grasp}")
                    self._grasp_lost = ctrlCommand.getFrameNames()

        elif ctrlCommand.getCommand() == ry.SC.OPEN_GRIPPER and not ctrlCommand.isCondition():
            print("I wanna open")
            if ctrlCommand.getFrameNames()[0] in self.gripper2index:
                gripper_index = self.gripper2index[ctrlCommand.getFrameNames()[0]]
                self.botop.gripperOpen(gripper_index, 1, 0.1)

                while not self.botop.gripperDone(gripper_index):
                    time.sleep(0.1)

                # Cheating, but for now always move up after placing something
                self._move_away_safely(*ctrlCommand.getFrameNames())

        elif ctrlCommand.isCondition():
            # return
            # check if still grasping:
            if ctrlCommand.getCommand() == ry.SC.CLOSE_GRIPPER:
                if ctrlCommand.getFrameNames()[0] in self.gripper2index:
                    gripper_index = self.gripper2index[ctrlCommand.getFrameNames()[0]]
                    self.botop.gripperClose(gripper_index, 0.01, 0.03, 0.1)
                    lost_grasp = self.botop.gripperGraspLost(gripper_index)
                    if lost_grasp:
                        print(f"Lost grasp: {lost_grasp}")
                        self._grasp_lost = ctrlCommand.getFrameNames()

    def shutdown(self):

        del self.botop

class RLGS(SimpleSystem):
    """
    Model Execution Class for this Thesis
    """

    def __init__(self, C,
                 use_real_robot=False,
                 use_feasy=True,
                 use_tracking=False,
                 verbose=False,
                 show_plots=False):

        # what the robot knows
        super().__init__(C, use_real_robot=use_real_robot,
                         show_plots=show_plots,
                         verbose=verbose,
                         use_tracking=use_tracking)

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
        self.feasy_check_rate = 3  # every second steps check for feasibility
        self.last_feasy_check = 0

    def init_system(self, action_list, planner, scene_objects):
        t_start = time.time()
        super().init_system(action_list, planner, scene_objects)

        # del self.active_plan

        # get robust tree/ set of chains
        self.robust_set_of_chains = get_robust_set_of_chains(self.C, self.action_tree, self.goal_controller,
                                                             verbose=self.show_plots)
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

        draw_search_graph(self.action_tree, "action_tree.png")

        return True, time.time() - t_start

    def is_no_plan_feasible(self):
        return self.no_plan_feasible

    def _step(self, t):

        q_real = self.C.getJointState()
        q_target = q_real

        self._check_if_still_grasping()

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        is_any_controller_feasible = False
        is_current_plan_feasible = True

        self.current_active_controller_index = 0

        # iterate over each controller, check which can be started first
        for i, (edge, name, c) in enumerate(self.active_robust_reverse_plan):

            if c.canBeInitiated(ctrl, self.eqPrecision):
                self.log(f"Initiating: {name}")
                ctrl.set(c)
                is_any_controller_feasible = True
                self.current_active_controller_index = i

                q_dot = self.q - self.q_old
                ctrl.update(q_real, q_dot, self.C)
                q_target = ctrl.solve(self.C)

                self.q = q_real
                self.q_old = self.q_old

                # Handle symbolic commands here
                for ctrlCommand in c.getSymbolicCommands():
                    self._handle_symbolic_commands(ctrlCommand)

                break
            else:
                pass
                # self.log(f"Cannot be initiated: {name}")

        # always make a step
        self.botop.moveLeap(q_target, 2)
        self.botop.step(self.C, self.leap_step)

        # check feasibility of chain
        if self.use_feasy and \
                t - self.last_feasy_check > self.feasy_check_rate:
            # check rest of controller chain for feasibility
            self.log("Checking current residual chain")
            residual_plan = self.active_robust_reverse_plan[self.current_active_controller_index::-1]
            is_current_plan_feasible, _ = check_switch_chain_feasibility(self.C, residual_plan,
                                                                         self.goal_controller,
                                                                         self.scene_objects,
                                                                         verbose=False)
            self.last_feasy_check = t

        # if current plan is not feasible, check other plans
        if not is_current_plan_feasible:
            self.log("Current Plan is not feasible!")
            self.active_robust_reverse_plan = self.get_feasible_reverse_plan(ctrl)

            self.last_feasy_check = t
        if self.active_robust_reverse_plan is None:
            self.no_plan_feasible = True

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
                                                                             verbose=False)
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
        self.log("No plan is feasible!")
        return None

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

    def __init__(self, C, verbose=False, use_real_robot=False, use_tracking=False):
        super().__init__(C, use_feasy=False, verbose=verbose, use_real_robot=use_real_robot, use_tracking=use_tracking)
