import libry as ry
from pyddl import Action, neg
from objective import Objective
import util.domain_tower as dt
import itertools as it


def _get_unset_effects(predicate, all_objects, obj_type):
    obj_count = len(all_objects[obj_type]) - 1

    obj_place_holder = [chr(ord("Z") - x) for x in range(obj_count)]

    unset_effects = [neg((predicate, x)) for x in obj_place_holder]

    params = [(obj_type, x) for x in obj_place_holder]

    return params, unset_effects


def _get_sym2frame(symbols, frames):
    return {sym: frames[i] for i, sym in enumerate(symbols.keys())}


# def get_komo(C, ctrlset):


class BaseController:

    def __init__(self, name):
        self.name = name
        self.symbols = None
        self.ctrl_set = None
        self.sym2frame = None

        self.objectives = []

    def set_frame_type_count(self):
        self.frame_type_count = \
            {obj_type: self.frame_type.count(obj_type) for obj_type in set(self.frame_type)}

    def getAllObjectives(self):
        return self.objectives

    def getImmediateObjectives(self):
        return [obj for obj in self.o]

    def get_description(self):
        for objective in self.objectives:
            print(objective.objective2symbol())

    def get_simple_action(self, all_objects=None):
        # This needs to be implemented for each controller
        print(f"This function needs to be implemented! {self.__name__}")

    def get_grounded_control_set(self, C, frames):
        self.ctrl_set = ry.CtrlSet()
        self.sym2frame = {sym: frames[i] for i, sym in enumerate(self.symbols.keys())}


class CloseGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.symbols = {
            "G": dt.type_gripper,
            "B": dt.type_block,
        }

        self.params = tuple({v: k for k, v in self.symbols.items()}.items())

    def get_simple_action(self, all_objects=None):
        return Action(self.name,
                      parameters=(
                          (dt.type_gripper, "g"),
                          (dt.type_block, "b"),
                      ),
                      preconditions=(
                          (dt.focus, "b"),
                          (dt.hand_empty, "g"),
                          (dt.block_free, "b"),
                      ),
                      effects=(
                          (dt.in_hand, "b", "g"),
                          neg((dt.hand_empty, "g")),
                          neg((dt.block_free, "b"))
                      ),
                      unique=True)

    def get_grounded_control_set(self, C, frames):
        ctrl_set = ry.CtrlSet()
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame['G']
        gripper_center = gripper + "Center"
        block = sym2frame['B']

        # ctrl_set.addObjective(
        #     C.feature(
        #         ry.FS.positionRel,
        #         [gripper_center, block],
        #         [1e1] * 3,
        #         [.0, 0., .05]),
        #     ry.OT.eq,
        #     -1)
        # # align axis with block
        # ctrl_set.addObjective(
        #     C.feature(
        #         ry.FS.vectorZDiff,
        #         [gripper, block],
        #         [1e1]),
        #     ry.OT.eq,
        #     -1)

        gripper_start_pos = C.frame(gripper).getPosition()

        #  move close to block
        ctrl_set.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e0]),
            ry.OT.eq, -1)
        #align axis with block
        ctrl_set.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), False)

        #ctrl_set.addObjective(C.feature(ry.FS.accumulatedCollisions, [], [1e1]), ry.OT.eq)

        return ctrl_set


class OpenGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.symbols = {
            "G": dt.type_gripper,
            "B": dt.type_block,
        }

    def get_simple_action(self, all_objects=None):
        return Action(self.name,
                      parameters=(
                          (dt.type_gripper, "g"),
                          (dt.type_block, "b")
                      ),
                      preconditions=(
                          (dt.focus, "b"),
                          (dt.in_hand, "b", "g")
                      ),
                      effects=(
                          neg((dt.in_hand, "b", "g")),
                          (dt.hand_empty, "g"),
                          (dt.block_free, "b")
                      ),
                      unique=True)

    def get_grounded_control_set(self, C, frames):
        ctrl_set = ry.CtrlSet()
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame['G']
        block = sym2frame['B']
        gripper_center = gripper + "Center"


        # ctrl_set.addObjective(
        #     C.feature(ry.FS.insideBox, [block, "R_gripperPregrasp"], [1e0]),
        #     ry.OT.eq, -1)

        ctrl_set.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        ctrl_set.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        ctrl_set.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        return ctrl_set


class ApproachBlock(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.symbols = {
            "G": dt.type_gripper,
            "B": dt.type_block,
        }

    def get_simple_action(self, all_objects=None):
        # special case, where we need to unset focus on other objects
        unset_params_focus, unset_effects_focus = _get_unset_effects(dt.focus, all_objects, dt.type_block)

        # Need

        return Action(self.name,
                      parameters=(
                          (dt.type_gripper, "g"),
                          (dt.type_block, "b"),
                          *unset_params_focus,
                      ),
                      preconditions=(
                          (dt.hand_empty, "g"),
                      ),
                      effects=(
                          (dt.focus, "b"),
                          *unset_effects_focus,
                      ),
                      unique=True)

    def get_grounded_control_set(self, C, frames):
        ctrl_set = ry.CtrlSet()
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame['G']
        gripper_center = gripper + "Center"
        block = sym2frame['B']

        # move close to block
        ctrl_set.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [1e1] * 3, [.0, 0., .05]),
            ry.OT.sos, 0.005)
        # align axis with block
        ctrl_set.addObjective(
            C.feature(ry.FS.vectorZDiff, [gripper, block], [1e1]),
            ry.OT.sos, 0.01)



        return ctrl_set


class PlaceOn(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.symbols = {
            "G": dt.type_gripper,
            "B": dt.type_block,
            "B_place_on": dt.type_block
        }

    def get_simple_action(self, all_objects=None):
        return Action(self.name,
                      parameters=(
                          (dt.type_gripper, "g"),
                          (dt.type_block, "b"),
                          (dt.type_block, "b_placed")
                      ),
                      preconditions=(
                          (dt.focus, "b"),
                          (dt.in_hand, "b", "g"),
                          (dt.block_free, "b_placed")
                      ),
                      effects=(
                          (dt.b_on_b, "b", "b_placed"),
                          neg((dt.block_free, "b_placed")),
                      ),
                      unique=True)

    def get_grounded_control_set(self, C, frames):
        ctrl_set = ry.CtrlSet()
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame['G']
        gripper_center = gripper + "Center"
        block = sym2frame['B']
        block_place_on = sym2frame['B_place_on']

        # block should be over block_placed_on
        ctrl_set.addObjective(
            C.feature(ry.FS.positionRel, [block, block_place_on], [1e1], [0, 0, 0.11]),
            ry.OT.sos, 0.005)
        # should have z-axis in same direction
        ctrl_set.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, block_place_on], [1e1], [1]),
            ry.OT.sos, 0.005)
        # align axis with block
        ctrl_set.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # not working between objects
        #ctrl_set.addObjective(C.feature(ry.FS.accumulatedCollisions, [], [1e1]), ry.OT.eq)

        # ctrl_set.addObjective(
        #     C.feature(ry.FS.distance, ["b2", "b1"], [1e1], [0.5]),
        #     ry.OT.ineq, -1)


        return ctrl_set
