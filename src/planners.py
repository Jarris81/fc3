from pyddl import Domain, Problem, State, Action, neg, planner, backwards_planner
import actions
import predicates as pred
import util.constants as dt
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

    def get_plan(self, control_actions, scene_obj, forward=False):

        # get simple action from all controllers
        domain = Domain((x.get_simple_action() for x in control_actions))

        # initial conditions
        init = list()

        # empty hand
        init_hand_empty = pred.HandEmpty("G")
        init_hand_empty.ground_predicate(G=scene_obj[dt.type_gripper][0])
        init.append(init_hand_empty.get_grounded_predicate())

        # free blocks
        for block in scene_obj[dt.type_block]:
            free_block = pred.BlockFree("B")
            free_block.ground_predicate(B=block)
            init.append(free_block.get_grounded_predicate())

        # normal goal predicates
        self.goal = list()

        # hand should be free
        goal_hand_empty = pred.HandEmpty("G")
        goal_hand_empty.ground_predicate(G=scene_obj[dt.type_gripper][0])
        self.goal.append(goal_hand_empty.get_grounded_predicate())

        # blocks on blocks
        for i in range(len(scene_obj[dt.type_block]) - 1):
            block_on_block = pred.BlockOnBlock("B", "B_placed")
            block_on_block.ground_predicate(B=scene_obj[dt.type_block][i], B_placed=scene_obj[dt.type_block][i+1])
            self.goal.append(block_on_block.get_grounded_predicate())

        # define problem here
        prob = Problem(
            domain,
            scene_obj,
            init=init,
            goal=self.goal
        )

        G = None
        state_plan = None

        # generate plan
        if forward:
            plan = planner(prob, verbose=self.verbose)
        else:
            action_plan, state_plan, __ = backwards_planner(prob, goal=self.goal, action_tree=False, verbose=True)
            plan, state_plan, G = backwards_planner(prob, goal=self.goal, action_tree=True, max_diff=1,
                                                           root_state_plan=state_plan, verbose=True)
            # need to reverse plan
            plan = plan[::-1]
        if self.verbose:
            if plan is None:
                print('No Plan!')
            else:
                for action in plan:
                    print(action)

        return plan, self.goal, state_plan, G

    def get_goal_controller(self, C):
        goal_feature = ry.CtrlSet()

        goals_block_on_block = [x for x in self.goal if x[0] == pred.BlockOnBlock.__name__]

        block_heights = dict()
        # cut out last goal, which was free hand
        for _, block, block_place_on in goals_block_on_block:

            if block not in block_heights:
                block_heights[block] = C.frame(block).getSize()[-2]
            if block_place_on not in block_heights:
                block_heights[block_place_on] = C.frame(block_place_on).getSize()[-2]

            height_block = block_heights[block]
            height_block_place_on = block_heights[block_place_on]

            dist = (height_block + height_block_place_on)/2

            goal_feature.addObjective(
                C.feature(ry.FS.positionRel, [block, block_place_on], [1e1], [0, 0, dist]),
                ry.OT.eq, -1)
            # should have z-axis in same direction
            goal_feature.addObjective(
                C.feature(ry.FS.vectorZDiff, [block, block_place_on], [1e1]),
                ry.OT.eq, -1)

            goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", block), True)

        return goal_feature


class PickAndPlacePlanner:

    def __init__(self, verbose=False):

        self.verbose = verbose
        self.goal = []

    def get_plan(self, control_actions, scene_obj, forward=False):

        domain = Domain((x.get_simple_action() for x in control_actions))

        init = list()

        # empty hands
        for gripper in scene_obj[dt.type_gripper]:
            init_hand_empty = pred.HandEmpty("G")
            init_hand_empty.ground_predicate(G=gripper)
            init.append(init_hand_empty.get_grounded_predicate())

        # free blocks
        for block in scene_obj[dt.type_block]:
            free_block = pred.BlockFree("B")
            free_block.ground_predicate(B=block)
            init.append(free_block.get_grounded_predicate())

        # normal goal predicates
        self.goal = list()

        # hands should be free
        for gripper in scene_obj[dt.type_gripper]:
            goal_hand_empty = pred.HandEmpty("G")
            goal_hand_empty.ground_predicate(G=gripper)
            self.goal.append(goal_hand_empty.get_grounded_predicate())

        # first block should be at position
        block_at_goal = pred.BlockAtGoal("B")
        block_at_goal.ground_predicate(B=scene_obj[dt.type_block][0])
        self.goal.append(block_at_goal.get_grounded_predicate())

        # define problem here
        prob = Problem(
            domain,
            scene_obj,
            init=init,
            goal=self.goal
        )

        G = None
        state_plan = None

        # generate plan
        if forward:
            plan = planner(prob, verbose=self.verbose)
        else:
            action_plan, state_plan, __ = backwards_planner(prob, goal=self.goal, action_tree=False, verbose=True)
            plan, state_plan, G = backwards_planner(prob, goal=self.goal, action_tree=True, max_diff=4,
                                                    root_state_plan=state_plan, verbose=True)
            # need to reverse plan
            plan = plan[::-1]
        if self.verbose:
            if plan is None:
                print('No Plan!')
            else:
                for action in plan:
                    print(action)

        return plan, self.goal, state_plan, G

    def get_goal_controller(self, C):
        goal_feature = ry.CtrlSet()
        # TODO put this in some domain class
        goal_place = (0.3, 0.3, 0.71)

        goals_block_at_goal = [x for x in self.goal if x[0] == pred.BlockAtGoal.__name__]

        for _, block in goals_block_at_goal:
            goal_feature.addObjective(
                C.feature(ry.FS.position, [block], [1e1], goal_place),
                ry.OT.eq, -1)
            goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("R_gripper", block), True)
            goal_feature.addSymbolicCommand(ry.SC.OPEN_GRIPPER, ("L_gripper", block), True)

        return goal_feature

class HandOverPlaner:

    def __init__(self, verbose=False):

        self.verbose = verbose
        self.goal = []

if __name__ == '__main__':
    from optparse import OptionParser

    parser = OptionParser(usage="Usage: %prog [options]")
    parser.add_option('-q', '--quiet',
                      action='store_false', dest='verbose', default=True,
                      help="don't print statistics to stdout")

    action_list = [
        actions.GrabBlock(),
        actions.PlacePosition(),
        actions.HandOver(),
    ]

    objects = {
             dt.type_block: (1,),  # , "b3"),
             dt.type_gripper: ("R_gripper", "L_gripper")
         }

    # Parse arguments
    opts, args = parser.parse_args()
    planner = PickAndPlacePlanner()
    plan, goal, state_plan, G = planner.get_plan(action_list, objects)

    for a in plan:
        print(a)

    draw_search_graph(plan, state_plan, G)


