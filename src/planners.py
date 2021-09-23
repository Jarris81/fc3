from pyddl import Domain, Problem, State, Action, neg, planner, backwards_planner, backwards_tree_exploration
import actions
import predicates as pred
from util import constants
import libry as ry
from util.visualize_search import draw_search_graph

"""
Simple planner to build a tower, which uses (simple) symbolic actions (STRIPS style)
"""


# TODO: add block order


class TowerPlanner:

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.goal = []

    def get_tree(self, control_actions, scene_obj, forward=False):

        # get simple action from all controllers
        domain = Domain((x.get_simple_action() for x in control_actions))

        # initial conditions
        init = list()

        # empty hand
        init_hand_empty = pred.HandEmpty("G")
        init_hand_empty.ground_predicate(G=scene_obj[constants.type_gripper][0])
        init.append(init_hand_empty.get_grounded_predicate())

        # free blocks
        for block in scene_obj[constants.type_block]:
            free_block = pred.IsFree("B")
            free_block.ground_predicate(B=block)
            init.append(free_block.get_grounded_predicate())

        # normal goal predicates
        self.goal = list()

        # hand should be free
        goal_hand_empty = pred.HandEmpty("G")
        goal_hand_empty.ground_predicate(G=scene_obj[constants.type_gripper][0])
        self.goal.append(goal_hand_empty.get_grounded_predicate())

        # blocks on blocks
        for i in range(len(scene_obj[constants.type_block]) - 1):
            block_on_block = pred.BlockOnBlock("B", "B_placed")
            block_on_block.ground_predicate(B=scene_obj[constants.type_block][i],
                                            B_placed=scene_obj[constants.type_block][i + 1])
            self.goal.append(block_on_block.get_grounded_predicate())

        # define problem here
        prob = Problem(
            domain,
            scene_obj,
            init=init,
            goal=self.goal
        )

        # generate plan
        if forward:
            action_tree = planner(prob, verbose=self.verbose)
        else:
            action_plan, state_plan1, __ = backwards_planner(prob, goal=self.goal, action_tree=False, verbose=True)
            plan, state_plan1, action_tree = backwards_planner(prob, goal=self.goal, action_tree=True, max_diff=1,
                                                               root_state_plan=state_plan1, verbose=True)
            # need to reverse plan
        if self.verbose:
            if action_tree is None:
                print('No Plan!')
        return action_tree

    def get_goal_controller(self, C):
        goal_feature = ry.CtrlSet()

        goals_block_on_block = [x for x in self.goal if x[0] == pred.BlockOnBlock.__name__]

        block_heights = dict()
        # cut out last goal, which was free hand
        for _, block, block_place_on in goals_block_on_block:

            if block not in block_heights:
                block_heights[block] = C.frame(block).getSize()[2]
            if block_place_on not in block_heights:
                block_heights[block_place_on] = C.frame(block_place_on).getSize()[2]

            height_block = block_heights[block]
            height_block_place_on = block_heights[block_place_on]

            dist = (height_block + height_block_place_on) / 2

            goal_feature.addObjective(
                C.feature(ry.FS.positionRel, [block, block_place_on], [1], [0, 0, dist]),
                ry.OT.eq, -1)
            # should have z-axis in same direction
            # goal_feature.addObjective(
            #     C.feature(ry.FS.vectorZDiff, [block, block_place_on], [1e1]),
            #     ry.OT.eq, -1)

            goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("r_gripper", block), True)

        return goal_feature


class HandOverPlanner:

    def __init__(self, verbose=False):

        self.verbose = verbose
        self.goal = []

    def get_tree(self, control_actions, scene_obj, forward=False):

        domain = Domain((x.get_simple_action() for x in control_actions))

        init = list()

        # hack, remove the left gripper from scene objects for forward approaches
        if forward:
            scene_obj[constants.type_gripper].remove("l_gripper")

        # empty hands
        for gripper in scene_obj[constants.type_gripper]:
            init_hand_empty = pred.HandEmpty("G")
            init_hand_empty.ground_predicate(G=gripper)
            init.append(init_hand_empty.get_grounded_predicate())

        # free blocks
        for block in scene_obj[constants.type_block]:
            free_block = pred.IsFree("B")
            free_block.ground_predicate(B=block)
            init.append(free_block.get_grounded_predicate())

        # normal goal predicates
        self.goal = list()

        # first block should be at position
        block_at_goal = pred.BlockAtGoal("B")
        block_at_goal.ground_predicate(B=scene_obj[constants.type_block][0])
        self.goal.append(block_at_goal.get_grounded_predicate())

        # define problem here
        prob = Problem(
            domain,
            scene_obj,
            init=init,
            goal=self.goal
        )

        action_tree = None

        # generate plan
        if forward:
            action_tree = planner(prob, verbose=self.verbose)
            # hack: check which

        else:
            # action_plan, state_plan, __ = backwards_planner(prob, goal=self.goal, action_tree=False, verbose=True)
            # plan, state_plan, action_tree = backwards_planner(prob, goal=self.goal, action_tree=True, max_diff=5,
            #                                         root_state_plan=state_plan, verbose=True)

            action_tree = backwards_tree_exploration(prob, goal=self.goal, verbose=True, max_depth=3)

        return action_tree

    def get_goal_controller(self, C):
        goal_controller = ry.CtrlSet()
        # TODO put this in some domain class
        goal_place = constants.goal_handover_block_pos
        constants.goal_block_pos = goal_place
        print(constants.goal_block_pos)

        goals_block_at_goal = [x for x in self.goal if x[0] == pred.BlockAtGoal.__name__]
        block = "b1"
        goal_controller.addObjective(
            C.feature(ry.FS.position, [block], [1e0, 1e0, 0], goal_place),
            ry.OT.eq, -1)
        goal_controller.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("r_gripper", block), True)
        goal_controller.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("l_gripper", block), True)

        return goal_controller


class StickPullPlanner:

    def __init__(self, verbose=False):

        self.verbose = verbose
        self.goal = []

    def get_tree(self, control_actions, scene_obj, forward=False):

        domain = Domain((x.get_simple_action() for x in control_actions))

        init = list()

        # empty hands
        for gripper in scene_obj[constants.type_gripper]:
            init_hand_empty = pred.HandEmpty("G")
            init_hand_empty.ground_predicate(G=gripper)
            init.append(init_hand_empty.get_grounded_predicate())

        # free blocks
        for block in scene_obj[constants.type_block]:
            free_block = pred.IsFree("B")
            free_block.ground_predicate(B=block)
            init.append(free_block.get_grounded_predicate())

        # normal goal predicates
        self.goal = list()
        block_at_goal = pred.BlockAtGoal("B")
        block_at_goal.ground_predicate(B=scene_obj[constants.type_block][0])
        self.goal.append(block_at_goal.get_grounded_predicate())

        # empty hands
        for gripper in scene_obj[constants.type_gripper]:
            init_hand_empty = pred.HandEmpty("G")
            init_hand_empty.ground_predicate(G=gripper)
            self.goal.append(init_hand_empty.get_grounded_predicate())

        # define problem here
        prob = Problem(
            domain,
            scene_obj,
            init=init,
            goal=self.goal
        )

        # generate plan
        if forward:
            action_tree = planner(prob, verbose=self.verbose)  # is only path actually
        else:
            action_plan, state_plan, __ = backwards_planner(prob, goal=self.goal, action_tree=False, verbose=True)
            plan, state_plan, action_tree = backwards_planner(prob, goal=self.goal, action_tree=True, max_diff=5,
                                                              root_state_plan=state_plan, verbose=True)

            action_tree = backwards_tree_exploration(prob, goal=self.goal, verbose=True, max_depth=3)
            # need to reverse plan
            plan = plan[::-1]
        if self.verbose:
            if plan is None:
                print('No Plan!')
            else:
                for action in plan:
                    print(action)

        return action_tree

    def get_goal_controller(self, C):
        goal_feature = ry.CtrlSet()
        # TODO put this in some domain class
        goal_place = constants.goal_stick_pull_block_pos
        constants.goal_block_pos = goal_place
        goals_block_at_goal = [x for x in self.goal if x[0] == pred.BlockAtGoal.__name__]
        block = "b1"
        stick = "stick"
        goal_feature.addObjective(
            C.feature(ry.FS.position, [block], [1e0, 1e0, 0], goal_place),
            ry.OT.eq, -1)
        goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("r_gripper", stick), True)
        goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (stick, block), True)
        goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("r_gripper", block), True)

        return goal_feature


if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    action_list = [
        actions.GrabBlock(),
        actions.PlaceGoal(),
        actions.PullBlockToGoal(),
        actions.GrabStick()
    ]

    objects = {
        constants.type_block: (1,),  # , "b3"),
        constants.type_gripper: ("R_gripper",),
        constants.type_stick: ("stick",)
    }

    # Parse arguments
    opts, args = parser.parse_args()
    planner = StickPullPlanner()
    plan, goal, state_plan, G = planner.get_tree(action_list, objects)

    for a in plan:
        print(a)

    draw_search_graph(plan, state_plan, G)
