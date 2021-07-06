import libry as ry
from pyddl import Action, neg
import predicates as pred
import util.constants as dt


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


def add_action_name(name, controller_tuples):
    return [(f"{name}_{x[0]}", x[1]) for x in controller_tuples]


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

    def getAllObjectives(self):
        return self.objectives

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

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]

        height_block = C.getFrame(block).getSize()[-1]

        transient_step = 0.02

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [1e2], [0, 0, height_block]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, gripper], [1e1]),
            ry.OT.sos, transient_step)

        cage_block = ry.CtrlSet()
        cage_block.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, block], [5, 5, 0]),
            ry.OT.eq, -1)
        # move close to block
        cage_block.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_center, block], [1e2]),
            ry.OT.sos, 0.005)
        cage_block.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, gripper], [1e1]),
            ry.OT.sos, transient_step)
        # align axis with block

        # weird: following objective lets the robot oscillate between align and cage
        # cage_block.addObjective(
        #     C.feature(ry.FS.vectorZDiff, [block, gripper], [1e1]),
        #     ry.OT.eq, -1)
        # cage_block.addObjective(
        #     C.feature(ry.FS.scalarProductYZ, [block, gripper], [1e1]),
        #     ry.OT.eq, -1)
        # cage_block.addObjective(
        #     C.feature(ry.FS.scalarProductYZ, [block, gripper], [1e1]),
        #     ry.OT.eq, -1)

        #  block needs to be close to block
        grab = ry.CtrlSet()
        grab.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_center, block], [1e0]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), False)

        # return tuple of controllers
        controllers = [("align_over", align_over),
                       ("cage_block", cage_block),
                       ("grab", grab)]

        return add_action_name(self.name, controllers)


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

        height_block = C.getFrame(block).getSize()[-2]
        height_block_place_on = C.getFrame(block_placed_on).getSize()[-2]

        dist = (height_block + height_block_place_on) / 2
        dist2 = dist*2

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e1]),
            ry.OT.sos, 0.01)
        align_over.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e1], [0, 0, dist2]),
            ry.OT.sos, 0.01)
        align_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        place_on_block = ry.CtrlSet()
        # place_on_block.addObjective(
        #     C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e1]),
        #     ry.OT.eq, -1)
        place_on_block.addObjective(
            C.feature(ry.FS.positionDiff, [block, block_placed_on], [5, 5, 0]),
            ry.OT.eq, -1)
        place_on_block.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e1], [0., 0., dist]),
            ry.OT.sos, 0.01)
        # should have z-axis in same direction
        place_on_block.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e1]),
            ry.OT.sos, 0.01)
        # align axis with block
        place_on_block.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        open_gripper.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [5], [0., 0., dist]),
            ry.OT.eq, -1)
        # open_gripper.addObjective(
        #     C.feature(ry.FS.distance, [gripper_center, block], [1e0]),
        #     ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        # return tuple of controllers
        controllers = [
            ("align_over", align_over),
            ("place_on_block", place_on_block),
            ("open_gripper", open_gripper)
        ]
        return add_action_name(self.name, controllers)


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
        free_place = (0.5, 0.1, 0.68)
        table_pos = (-0.2, -0.2, 0)
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
            C.feature(ry.FS.position, [block], [1e3], free_place),
            ry.OT.sos, 0.005)

        place_block_side.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "table"], [1e1], [1]),
            ry.OT.sos, 0.005)

        # needs to be holding the block
        place_block_side.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        # open_gripper.addObjective(
        #     C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
        #     ry.OT.eq, -1)

        # necessary, so that the block is only released when on ground, and not mid-air
        open_gripper.addObjective(
            C.feature(ry.FS.position, [block], [1e0], free_place),
            ry.OT.eq, -1)
        # open_gripper.addObjective(
        #     C.feature(ry.FS.scalarProductZZ, [block, "world"], [1e1], [1]),
        #     ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        # return tuple of controllers
        controllers = [place_block_side, open_gripper]
        controller_names = ["place_block_side", "open_gripper"]

        controllers = list(zip(controller_names, controllers))

        return add_action_name(self.name, controllers)


class PlacePosition(BaseAction):

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
            pred.BlockAtGoal(self.block_sym),
            pred.BlockFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        goal_place = (0.3, 0.3, 0.71)
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]

        place_block_pos = ry.CtrlSet()
        place_block_pos.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        # block should be placed on table, doesnt matter where in x-y plane
        place_block_pos.addObjective(
            C.feature(ry.FS.position, [block], [1e2], goal_place),
            ry.OT.sos, 0.01)

        place_block_pos.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "world"], [1e1], [1]),
            ry.OT.sos, 0.01)

        # needs to be holding the block
        place_block_pos.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        open_gripper.addObjective(
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
            ry.OT.eq, -1)

        # necessary, so that the block is only released when on ground, and not mid-air
        open_gripper.addObjective(
            C.feature(ry.FS.position, [block], [0, 0, 1], goal_place),
            ry.OT.eq, -1)
        open_gripper.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "world"], [1e1], [1]),
            ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        controllers = [
            ("place_block_pos", place_block_pos),
            ("open_gripper", open_gripper)
        ]
        return add_action_name(self.name, controllers)


class GrabStick(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.stick_sym = "S"

        self.symbols_types = {
            self.gripper_sym: dt.type_gripper,
            self.stick_sym: dt.type_stick,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.HandEmpty(self.gripper_sym),
            pred.BlockFree(self.stick_sym)
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.stick_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        stick = sym2frame[self.stick_sym]

        # get stick length
        stick_frame = C.getFrame(stick)
        stick_length = stick_frame.getSize()[0] - 0.02

        move_to = ry.CtrlSet()
        # move close to block
        move_to.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, stick], [1e1] * 3, [0, 0, stick_length / 2]),
            ry.OT.sos, 0.005)
        # align axis with block
        move_to.addObjective(
            C.feature(ry.FS.scalarProductXZ, [gripper, stick], [1e1]),
            ry.OT.sos, 0.01)

        #  block needs to be close to block
        grab = ry.CtrlSet()
        grab.addObjective(
            C.feature(ry.FS.positionRel, [gripper_center, stick], [1e1] * 3, [0, 0, stick_length / 2]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, stick), True)
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), False)

        # return tuple of controllers
        return [move_to, grab]


class PullBlockStick(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"
        self.stick_sym = "S"

        self.symbols_types = {
            self.gripper_sym: dt.type_gripper,
            self.block_sym: dt.type_block,
            self.stick_sym: dt.type_stick,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.stick_sym),
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.stick_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]
        stick = sym2frame[self.stick_sym]
        stick_handle = stick + "Handle"

        # get stick length

        move_to_block = ry.CtrlSet()
        # move close to block
        move_to_block.addObjective(
            C.feature(ry.FS.positionRel, [stick_handle, block], [1e1] * 3, [0, 0.1, 0]),
            ry.OT.sos, 0.005)
        # align axis with block
        # move_to_block.addObjective(
        #     C.feature(ry.FS.scalarProductXZ, [gripper, stick], [1e1]),
        #     ry.OT.sos, 0.01)

        #  block needs to be close to block
        # grab = ry.CtrlSet()
        # grab.addObjective(
        #     C.feature(ry.FS.positionRel, [gripper_center, stick], [1e1] * 3, [0, 0, stick_length / 2]),
        #     ry.OT.eq, -1)
        # # condition, nothing is in hand of gripper
        # grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, stick), True)
        # grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), False)

        # return tuple of controllers
        return [move_to_block]


class HandOver(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_1_sym = "G1"
        self.gripper_2_sym = "G2"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_1_sym: dt.type_gripper,
            self.gripper_2_sym: dt.type_gripper,
            self.block_sym: dt.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_1_sym, self.block_sym),
            pred.HandEmpty(self.gripper_2_sym),
        ]

        self.add_effects = [
            pred.InHand(self.gripper_2_sym, self.block_sym),
            pred.HandEmpty(self.gripper_1_sym),
        ]
        # self.delete_effects = []

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        hand_over_pos_1 = [0, 0.0, 1.2]
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper_1 = sym2frame[self.gripper_1_sym]
        gripper_1_center = gripper_1 + "Center"

        gripper_2 = sym2frame[self.gripper_2_sym]
        gripper_2_center = gripper_2 + "Center"

        block = sym2frame[self.block_sym]

        align_1 = ry.CtrlSet()

        align_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, block), True)

        align_1.addObjective(
            C.feature(ry.FS.position, [block], [1e2], hand_over_pos_1),
            ry.OT.sos, 0.005)
        align_1.addObjective(
            C.feature(ry.FS.vectorZ, [gripper_1], [1e1], [1, 0, 0]),
            ry.OT.sos, 0.01)

        align_2 = ry.CtrlSet()
        align_2.addObjective(
            C.feature(ry.FS.vectorZ, [gripper_1], [1e1], [1, 0, 0]),
            ry.OT.eq, 0.01)
        align_2.addObjective(
            C.feature(ry.FS.positionRel, [gripper_2, gripper_1], [1e1], [0, 0, -0.3]),
            ry.OT.sos, 0.01)
        align_2.addObjective(
            C.feature(ry.FS.scalarProductZZ, [gripper_1, gripper_2], [1e2], [-1]),
            ry.OT.sos, 0.01)
        align_2.addObjective(
            C.feature(ry.FS.scalarProductXX, [gripper_1, gripper_2], [1e2]),
            ry.OT.sos, 0.01)
        align_2.addObjective(
            C.feature(ry.FS.position, [block], [1e1], hand_over_pos_1),
            ry.OT.eq, -1)

        cage = ry.CtrlSet()
        cage.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, block), True)
        # cage.addObjective(
        #     C.feature(ry.FS.distance, [gripper_1, gripper_2], [1e1], [0.25]),
        #     ry.OT.ineq, -1)
        cage.addObjective(
            C.feature(ry.FS.position, [block], [1e1], hand_over_pos_1),
            ry.OT.eq, -1)
        cage.addObjective(
            C.feature(ry.FS.scalarProductZZ, [gripper_1, gripper_2], [1e2], [-1]),
            ry.OT.eq, -1)
        cage.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_1_center, gripper_2_center], [1e2]),
            ry.OT.sos, 0.005)

        hand_over = ry.CtrlSet()
        hand_over.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_1_center, block], [1e2]),
            ry.OT.eq, -1)
        hand_over.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_2_center, block], [1e2]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        hand_over.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_1, block), False)
        hand_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, block), True)

        hand_over.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_2, block), True)
        hand_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_2, block), False)



        controllers = [
            ("align_1", align_1),
            ("align_2", align_2),
            ("cage", cage),
            ("hand_over", hand_over)
        ]

        return add_action_name(self.name, controllers)


