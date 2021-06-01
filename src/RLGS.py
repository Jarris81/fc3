from scipy.spatial.transform.rotation import Rotation
import libry as ry
import networkx as nx

import util.domain_tower as dt
from actions import GrabBlock, PlaceOn, PlaceSide
from testing.tower_planner import get_plan, get_goal_controller
from feasibility import check_switch_chain_feasibility
from robustness import get_robust_set_of_chains
from util.visualize_search import draw_search_graph


class RLGS:

    def __init__(self, C,
                 use_robust=True,
                 use_feasy=True,
                 use_multi=True,
                 verbose=False):

        # what the robot knows
        self.C = ry.Config()
        self.C.copy(C)
        self.use_feasy = use_feasy
        self.use_robust = use_robust
        self.use_multi = use_multi
        self.verbose = verbose

        # store plan and action tree here
        self.robust_set_of_chains = []
        self.active_robust_reverse_plan = []
        self.goal_controller = ry.CtrlSet()

        # stuff for execution
        self.is_done = False
        self.feasy_check_rate = 50  # every 50 steps check for feasibility
        self.C.view()

    def setup_tower(self, block_names):
        # get all actions needed to build a tower
        action_list = [
            GrabBlock(),
            PlaceOn(),
            PlaceSide()
        ]

        # setup config and get frame names
        gripper_name = "R_gripper"

        # put all objects into one dictionary with types
        scene_objects = {
            dt.type_block: block_names,
            dt.type_gripper: (gripper_name,)
        }

        # get plan, goal, state plan and the search tree
        plan, goal, state_plan, G = get_plan(self.verbose, action_list, scene_objects)

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
        grounded_actions = nx.get_edge_attributes(G, "action")
        grounded_ctrlsets = dict()

        for edge in G.edges():
            grounded_action = grounded_actions[edge]
            relevant_frames = grounded_action.sig[1:]
            controller = name2con[grounded_action.sig[0]]  # the actual controller
            grounded_ctrlsets[edge] = controller.get_grounded_control_set(self.C, relevant_frames)

        # each edge (action) gets and ctrlset
        nx.set_edge_attributes(G, grounded_ctrlsets, "ctrlset")

        # get goal controller, with only immediate conditions features (needed for feasibility)
        self.goal_controller = get_goal_controller(self.C, goal)

        # draw the action tree
        draw_search_graph(plan, state_plan, G)

        # get robust tree/ set of chains
        self.robust_set_of_chains = get_robust_set_of_chains(self.C, G, state_plan, self.goal_controller, False)

        # first plan we want to execute
        first_plan = self.robust_set_of_chains[0]

        # check if plan is feasible in current config
        is_feasible, komo_feasy = check_switch_chain_feasibility(self.C, first_plan, self.goal_controller, verbose=False)
        self.active_robust_reverse_plan = first_plan[::-1]

        if not is_feasible:
            print("Plan is not feasible in current Scene!")
            print("Aborting")
            return

    def is_done(self):
        return self.is_done

    def step(self, t, tau):

        # create a new solver every step (not ideal)
        ctrl = ry.CtrlSolver(self.C, tau, 2)
        is_any_controller_feasible = False
        is_current_plan_feasible = True
        is_any_plan_feasible = True

        # iterate over each controller, check which can be started first
        for i, (name, c) in enumerate(self.active_robust_reverse_plan):
            if c.canBeInitiated(self.C):
                ctrl.set(c)
                is_any_controller_feasible = True
                # check feasibility of chain
                if not t % self.feasy_check_rate:
                    # check rest of chain for feasibility
                    residual_plan = self.active_robust_reverse_plan[i::-1]
                    is_plan_feasible, _ = check_switch_chain_feasibility(self.C, residual_plan, self.goal_controller,
                                                                         verbose=False)
                if self.verbose:
                    print(f"Initiating: {name}")
                # leave loop, we have the controller
                break
            else:
                if self.verbose:
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

        ctrl.update(self.C)
        q = ctrl.solve(self.C)
        self.C.setJointState(q)

        return q
        #self.C.computeCollisions()
        # coll = C.getCollisions(0)
        #time.sleep(tau)

    def cheat_update_obj(self, object_infos):
        for obj_name, obj_info in object_infos.items():
            obj = self.C.frame(obj_name)
            if obj is None:
                obj = self.C.addFrame(obj_name)
            obj.setPosition(obj_info['pos'])
            size = obj_info['size']
            if len(size) == 3:
                obj.setShape(ry.ST.box, size=size)
            else:
                obj.setShape(ry.ST.ssBox, size=size)
            rot = Rotation.from_matrix(obj_info['rot_max'])
            obj.setQuaternion(rot.as_quat())