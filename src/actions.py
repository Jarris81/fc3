import libry as ry
from pyddl import Action, neg
import predicates as pred
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
    return {sym: frames[i] for i, sym in enumerate(symbols)}


def _get_grab_controller(C, gripper, grabing_object, grabbing=True):
    gripper_center = gripper + "Center"
    grab = ry.CtrlSet()
    gripper_preGrasp = gripper + "Pregrasp"
    # grab.addObjective(
    #     C.feature(ry.FS.distance, [grabing_object, gripper_center], [1e1]),
    #     ry.OT.eq, -1)
    grab.addObjective(
        C.feature(ry.FS.insideBox, [grabing_object, gripper_preGrasp], [1e0]),
        ry.OT.eq, -1)

    # condition, nothing is in hand of gripper
    grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, grabing_object), grabbing)
    grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, grabing_object), not grabbing)

    return grab


class BaseAction:

    def __init__(self, name):
        self.name = name
        self.symbols = None
        self.ctrl_set = None
        self.sym2frame = None
        self.symbols_types = None
        self.preconditions = None
        self.add_effects = None
        self.delete_effects = None

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

    def get_simple_action(self):
        # special case, where we need to unset focus on other objects

        return Action(self.name,
                      parameters=tuple(zip(self.symbols_types.values(), self.symbols_types.keys())),
                      preconditions=[x.get_predicate() for x in self.preconditions],
                      effects=[neg(x.get_predicate()) for x in self.delete_effects] +
                              [y.get_predicate() for y in self.add_effects],
                      unique=True)

    def get_grounded_control_set(self, C, frames):
        self.ctrl_set = ry.CtrlSet()
        self.sym2frame = {sym: frames[i] for i, sym in enumerate(self.symbols.keys())}


class GrabBlock(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_sym: dt.type_gripper,
            self.block_sym: dt.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.HandEmpty(self.gripper_sym),
            pred.BlockFree(self.block_sym)
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.block_sym)
        ]

        self.delete_effects = [
            self.preconditions[0]
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]

        align_over = ry.CtrlSet()
        # move close to block
        align_over.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [1e1] * 3, [.0, 0., .05]),
            ry.OT.sos, 0.005)
        # align axis with block
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, [gripper, block], [1e1]),
            ry.OT.sos, 0.01)

        move_close = ry.CtrlSet()
        move_close.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [1e1] * 3, [.0, 0., .15]),
            ry.OT.ineq, -1)
        move_close.addObjective(
            C.feature(ry.FS.vectorZDiff, [gripper, block], [1e1]),
            ry.OT.sos, 0.01)
        move_close.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [1e1] * 3, [.0, 0., .05]),
            ry.OT.sos, 0.005)

        #  block needs to be close to block
        grab = ry.CtrlSet()
        grab.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), False)

        # return tuple of controllers
        return [align_over, grab]


class PlaceOn(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"
        self.block_placed_sym = "B_placed"

        self.symbols_types = {
            self.gripper_sym: dt.type_gripper,
            self.block_sym: dt.type_block,
            self.block_placed_sym: dt.type_block
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.block_sym),
            pred.BlockFree(self.block_placed_sym)
        ]

        self.add_effects = [
            pred.BlockOnBlock(self.block_sym, self.block_placed_sym),
            pred.BlockFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]
        block_placed_on = sym2frame['B_placed']

        place_on_block = ry.CtrlSet()
        place_on_block.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e2]),
            ry.OT.eq, -1)
        # block should be over block_placed_on
        place_on_block.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e2]*3, [0., 0., .1]),
            ry.OT.sos, 0.005)
        # should have z-axis in same direction
        place_on_block.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, block_placed_on], [1e1], [1]),
            ry.OT.sos, 0.005)
        # align axis with block
        place_on_block.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        open_gripper.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        return [place_on_block, open_gripper]


class PlaceSide(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_sym: dt.type_gripper,
            self.block_sym: dt.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.block_sym),
        ]

        self.add_effects = [
            pred.BlockFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):

        free_place = (0., 0., 0.71)
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]

        place_block_side = ry.CtrlSet()
        place_block_side.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        # block should be placed on table, doesnt matter where in x-y plane
        place_block_side.addObjective(
            C.feature(ry.FS.position, [block], [0, 0, 1e1], free_place),
            ry.OT.sos, 0.005)

        place_block_side.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "world"], [1e1], [1]),
            ry.OT.sos, 0.005)

        # needs to be holding the block
        place_block_side.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        open_gripper.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        # necessary, so that the block is only released when on ground, and not mid-air
        open_gripper.addObjective(
            C.feature(ry.FS.position, [block], [0, 0, 1], free_place),
            ry.OT.eq, -1)
        open_gripper.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "world"], [1e1], [1]),
            ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        return [place_block_side, open_gripper]



