import libry as ry
from pyddl import Action, neg
import predicates as pred
from util import constants

transient_step = 0.1  # should be 0.1 default, so that interference work
world_frame = "world"


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
            self.gripper_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.HandEmpty(self.gripper_sym),
            pred.IsFree(self.block_sym)
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.block_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        block = sym2frame[self.block_sym]

        height_block = C.getFrame(block).getSize()[2]

        align_over_dist = 1.5 * height_block

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.positionRel, [gripper, block], [1e1], [0, 0, align_over_dist]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, ["world", gripper], [1e1]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.scalarProductXX, [block, gripper], [1e1]),
            ry.OT.sos, transient_step)
        align_over.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)

        cage_block = ry.CtrlSet()
        cage_block.addObjective(
            C.feature(ry.FS.positionRel, [gripper, block], [1e0, 1e0, 0]),
            ry.OT.eq, -1)
        # move close to block
        cage_block.addObjective(
            C.feature(ry.FS.positionDiff, [gripper, block], [1e2]),
            ry.OT.sos, transient_step / 5)
        cage_block.addObjective(
            C.feature(ry.FS.vectorZDiff, ["world", gripper], [1e1]),
            ry.OT.sos, transient_step / 5)
        cage_block.addObjective(
            C.feature(ry.FS.scalarProductXX, [block, gripper], [1e0]),
            ry.OT.eq, -1)
        cage_block.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)
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
            C.feature(ry.FS.positionDiff, [gripper, block], [1e0]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), False)

        # return tuple of controllers
        controllers = [
            ("align_over", align_over),
            ("cage_block", cage_block),
            ("grab", grab)
        ]

        return add_action_name(self.name, controllers)


class GrabBottle(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.bottle_sym = "B"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.bottle_sym: constants.type_bottle,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.HandEmpty(self.gripper_sym),
            pred.BottleFree(self.bottle_sym)
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.bottle_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        bottle = sym2frame[self.bottle_sym]

        bottle_radius = C.getFrame(bottle).getSize()[1]

        align_side = ry.CtrlSet()
        align_side.addObjective(
            C.feature(ry.FS.positionRel, [bottle, gripper_center], [1e2], [0, 0, -bottle_radius * 2]),
            ry.OT.sos, transient_step)
        align_side.addObjective(
            C.feature(ry.FS.scalarProductYZ, [gripper, bottle], [1e3], [-1]),
            ry.OT.sos, transient_step * 2)
        align_side.addObjective(
            C.feature(ry.FS.scalarProductXZ, [gripper, bottle], [1e3]),
            ry.OT.sos, transient_step)

        # move close to bottle and keep orientation
        cage_bottle = ry.CtrlSet()
        cage_bottle.addObjective(
            C.feature(ry.FS.scalarProductYZ, [gripper, bottle], [1e0], [-1]),
            ry.OT.eq, -1)
        cage_bottle.addObjective(
            C.feature(ry.FS.scalarProductXZ, [gripper, bottle], [1e0]),
            ry.OT.eq, -1)

        cage_bottle.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_center, bottle], [1e2]),
            ry.OT.sos, transient_step)
        # align axis with bottle
        grab = ry.CtrlSet()
        grab.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_center, bottle], [1e1]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, bottle), True)
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, bottle), False)

        # return tuple of controllers
        controllers = [
            ("align_side", align_side),
            ("cage_bottle", cage_bottle),
            ("grab", grab)
        ]

        return add_action_name(self.name, controllers)


class PlaceOn(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"
        self.block_placed_sym = "B_placed"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
            self.block_placed_sym: constants.type_block
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.block_sym),
            pred.IsFree(self.block_placed_sym)
        ]

        self.add_effects = [
            pred.BlockOnBlock(self.block_sym, self.block_placed_sym),
            pred.IsFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        block = sym2frame[self.block_sym]
        block_placed_on = sym2frame['B_placed']

        height_block = C.getFrame(block).getSize()[2]
        height_block_place_on = C.getFrame(block_placed_on).getSize()[2]

        dist = (height_block + height_block_place_on) / 2
        dist2 = dist * 2

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e1]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.positionDiff, [block, block_placed_on], [1e1], [0, 0, 2 * dist]),
            ry.OT.sos, transient_step)
        align_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        place_on_block = ry.CtrlSet()
        # place_on_block.addObjective(
        #     C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e1]),
        #     ry.OT.eq, -1)
        place_on_block.addObjective(
            C.feature(ry.FS.positionDiff, [block, block_placed_on], [1e0, 1e0, 0]),
            ry.OT.eq, -1)
        place_on_block.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e1], [0., 0., dist]),
            ry.OT.sos, transient_step / 5)
        # should have z-axis in same direction
        place_on_block.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, block_placed_on], [1e0]),
            ry.OT.sos, transient_step)
        # align axis with block
        place_on_block.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        open_gripper.addObjective(
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1], [0., 0., dist]),
            ry.OT.eq, -1)
        # open_gripper.addObjective(
        #     C.feature(ry.FS.distance, [gripper_center, block], [1e0]),
        #     ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        move_safely = ry.CtrlSet()
        move_safely.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, gripper], [1e1]),
            ry.OT.sos, 0.01)
        move_safely.addObjective(
            C.feature(ry.FS.positionRel, [gripper, block], [1e2], [0, 0, dist2]),
            ry.OT.sos, 0.01)
        move_safely.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), True)

        # return tuple of controllers
        controllers = [
            ("align_over", align_over),
            ("place_on_block", place_on_block),
            ("open_gripper", open_gripper),
            # ("move_safely", move_safely)
        ]
        return add_action_name(self.name, controllers)


class PlaceSide(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.block_sym),
        ]

        self.add_effects = [
            pred.IsFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        free_place = (0.4, 0.4, constants.table_height + 0.03)
        table_pos = (-0.2, -0.2, 0)
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        block = sym2frame[self.block_sym]

        place_block_side = ry.CtrlSet()
        place_block_side.addObjective(
            C.feature(ry.FS.positionDiff, [block, gripper], [1e-1]),
            ry.OT.eq, -1)

        # block should be placed on table, doesnt matter where in x-y plane
        place_block_side.addObjective(
            C.feature(ry.FS.position, [block], [1e1], free_place),
            ry.OT.sos, transient_step)

        place_block_side.addObjective(
            C.feature(ry.FS.scalarProductZZ, [block, "table"], [1e0], [1]),
            ry.OT.sos, transient_step)

        # needs to be holding the block
        place_block_side.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        # open_gripper.addObjective(
        #     C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
        #     ry.OT.eq, -1)

        # necessary, so that the block is only released when on ground, and not mid-air
        open_gripper.addObjective(
            C.feature(ry.FS.position, [block], [5e0], free_place),
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


class PlaceGoal(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.block_sym),
        ]

        self.add_effects = [
            pred.BlockAtGoal(self.block_sym),
            pred.IsFree(self.block_sym),
            pred.HandEmpty(self.gripper_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        goal_place = constants.goal_block_pos
        sym2frame = _get_sym2frame(self.symbols, frames)

        align_over_pos = list(goal_place)
        align_over_pos[2] += 0.06

        gripper = sym2frame[self.gripper_sym]
        block = sym2frame[self.block_sym]

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, "world"], [1e1]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.position, [block], [1e2], align_over_pos),
            ry.OT.sos, transient_step)

        align_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)



        place_block_pos = ry.CtrlSet()

        # block should be placed on table, doesnt matter where in x-y plane
        place_block_pos.addObjective(
            C.feature(ry.FS.position, [block], [1e0, 1e0, 0], goal_place),
            ry.OT.eq, -1)

        place_block_pos.addObjective(
            C.feature(ry.FS.vectorZDiff, [block, "world"], [1e1]),
            ry.OT.sos, transient_step/5)

        place_block_pos.addObjective(
            C.feature(ry.FS.position, [block], [1e2], goal_place),
            ry.OT.sos, transient_step/5)

        # place_block_pos.addObjective(
        #     C.feature(ry.FS.scalarProductYZ, [block, "world"], [1e1]),
        #     ry.OT.sos, speed)
        # place_block_pos.addObjective(
        #     C.feature(ry.FS.positionDiff, [block, gripper], [1e0]),
        #     ry.OT.eq, -1)

        # needs to be holding the block
        place_block_pos.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)

        # open gripper
        open_gripper = ry.CtrlSet()
        # open_gripper.addObjective(
        #     C.feature(ry.FS.distance, [block, gripper_center], [1e0]),
        #     ry.OT.eq, -1)

        # necessary, so that the block is only released when on ground, and not mid-air
        open_gripper.addObjective(
            C.feature(ry.FS.position, [block], [1e1], goal_place),
            ry.OT.eq, -1)
        # open_gripper.addObjective(
        #     C.feature(ry.FS.scalarProductZZ, [gripper, "world"], [1e1], [1]),
        #     ry.OT.eq, -1)

        open_gripper.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, block), True)
        open_gripper.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, block), False)

        # TODo add overhead

        controllers = [
            ("align_over" , align_over),
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
            self.gripper_sym: constants.type_gripper,
            self.stick_sym: constants.type_stick,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.HandEmpty(self.gripper_sym),
            pred.IsFree(self.stick_sym)
        ]

        self.add_effects = [
            pred.InHand(self.gripper_sym, self.stick_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper = sym2frame[self.gripper_sym]
        stick = sym2frame[self.stick_sym]

        # get stick length
        stick_frame = C.getFrame(stick)
        stick_length = stick_frame.getSize()[1]


        grab_pos = (0, -0.05, -0.01)
        align_pos = list(grab_pos)
        align_pos[2] += 0.06

        align_over = ry.CtrlSet()
        align_over.addObjective(
            C.feature(ry.FS.positionRel, [gripper, stick], [1e1], align_pos),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.vectorZDiff, ["world", gripper], [1e1]),
            ry.OT.sos, transient_step)
        align_over.addObjective(
            C.feature(ry.FS.scalarProductYX, [gripper, stick], [1e1]),
            ry.OT.sos, transientStep=transient_step)

        move_to = ry.CtrlSet()
        move_to.addObjective(
            C.feature(ry.FS.positionRel, [gripper, stick], [1e0, 1e0, 0], grab_pos),
            ry.OT.eq, transientStep=-1)
        # move close to block
        move_to.addObjective(
            C.feature(ry.FS.positionRel, [gripper, stick], [1e1], grab_pos),
            ry.OT.sos, transientStep=transient_step)
        # align axis with block
        move_to.addObjective(
            C.feature(ry.FS.vectorZDiff, ["world", gripper], [1e0]),
            ry.OT.eq, -1)
        move_to.addObjective(
            C.feature(ry.FS.scalarProductYX, [gripper, stick], [1e0]),
            ry.OT.eq, transientStep=-1)

        #  block needs to be close to block
        grab = ry.CtrlSet()
        grab.addObjective(
            C.feature(ry.FS.positionRel, [gripper, stick], [1e0], grab_pos),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), False)

        # return tuple of controllers
        # return [move_to, grab]

        controllers = [
            ("align_over", align_over),
            ("move_to_stick", move_to),
            ("grab", grab)
        ]
        return add_action_name(self.name, controllers)


class PullBlockToGoal(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.block_sym = "B"
        self.stick_sym = "S"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
            self.stick_sym: constants.type_stick,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.stick_sym),
        ]

        self.add_effects = [
            pred.BlockAtGoal(self.block_sym)
        ]

        self.delete_effects = []

    def get_grounded_control_set(self, C, frames):
        sym2frame = _get_sym2frame(self.symbols, frames)

        stick = sym2frame[self.stick_sym]

        # get stick length
        stick_frame = C.getFrame(stick)
        stick_length = stick_frame.getSize()[1]

        pull_angle = 0.5

        gripper = sym2frame[self.gripper_sym]
        block = sym2frame[self.block_sym]
        stick = sym2frame[self.stick_sym]
        stick_handle = stick + "Handle"

        block_pos_goal = constants.goal_block_pos
        # block_pos_goal[1] = block_pos_goal[1] - 0.3

        align_pos_rel = [-0.03, 0.05, 0.0]
        hover_pos_rel = list(align_pos_rel)
        hover_pos_rel[0] -= 0.01
        hover_pos_rel[1] += 0.01
        hover_pos_rel[2] += 0.1

        hover_stick = ry.CtrlSet()
        # move close to block
        hover_stick.addObjective(
            C.feature(ry.FS.positionDiff, [stick_handle, block], [1e1], hover_pos_rel),
            ry.OT.sos, transient_step / 2)

        # align axis with block

        hover_stick.addObjective(
            C.feature(ry.FS.vectorZDiff, ["world", stick], [1e1]),
            ry.OT.sos, transient_step)

        hover_stick.addObjective(
            C.feature(ry.FS.scalarProductXY, [stick_handle, "world"], [1e0], [pull_angle]),
            ry.OT.sos, transient_step)

        hover_stick.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), True)

        stick_to_block = ry.CtrlSet()

        # move close to block
        stick_to_block.addObjective(
            C.feature(ry.FS.positionDiff, [stick_handle, block], [1e2] * 3, align_pos_rel),
            ry.OT.sos, transient_step/2)

        stick_to_block.addObjective(
            C.feature(ry.FS.positionDiff, [stick_handle, block], [0.5, 0.5, 0], align_pos_rel),
            ry.OT.eq, -1)

        # align axis with block

        attach_handle_stick = ry.CtrlSet()
        attach_handle_stick.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (stick, block), False)
        attach_handle_stick.addObjective(
            C.feature(ry.FS.positionDiff, [stick_handle, block], [1e0], align_pos_rel),
            ry.OT.eq, -1)
        # attach_handle_stick.addObjective(
        #     C.feature(ry.FS.scalarProductXY, [stick_handle, "world"], [1e0], [pull_angle]),
        #     ry.OT.eq, -1)

        pull_back = ry.CtrlSet()
        pull_back.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (stick, block), True)

        pull_back.addObjective(
            C.feature(ry.FS.position, [block], [1e2], block_pos_goal),
            ry.OT.sos, transient_step)

        pull_back.addObjective(
            C.feature(ry.FS.scalarProductXY, [stick_handle, "world"], [1e0], [pull_angle]),
            ry.OT.sos, transient_step)

        detach_block = ry.CtrlSet()
        detach_block.addObjective(
            C.feature(ry.FS.position, [block], [0.01, 0.01, 0], block_pos_goal),
            ry.OT.eq, -1)

        detach_block.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (stick, block), False)

        for ctrl in [stick_to_block, attach_handle_stick, pull_back]:

            ctrl.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), True)
            # ctrl.addObjective(
            #     C.feature(ry.FS.scalarProductZZ, ["world", stick], [1e0], [1]),
            #     ry.OT.eq, -1)
            ctrl.addObjective(
                C.feature(ry.FS.vectorZDiff, ["world", stick_handle], [1e0]),
                ry.OT.eq, -1)

        # for ctrl in [attach_handle_stick, pull_back, detach_block]:
        #     ctrl.addObjective(
        #         C.feature(ry.FS.vectorZDiff, ["world", stick_handle], [1e-1]),
        #         ry.OT.eq, -1)
        #  block needs to be close to block
        # grab = ry.CtrlSet()
        # grab.addObjective(
        #     C.feature(ry.FS.positionRel, [gripper_center, stick], [1e1] * 3, [0, 0, stick_length / 2]),
        #     ry.OT.eq, -1)
        # # condition, nothing is in hand of gripper
        # grab.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, stick), True)
        # grab.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper, stick), False)

        # return tuple of controllers
        controllers = [
            ("hover_stick", hover_stick),
            ("stick_to_block", stick_to_block),
            ("attach_handle_stick", attach_handle_stick),
            ("pull_back", pull_back),
            ("detach_block", detach_block)
        ]
        return add_action_name(self.name, controllers)


class PlaceStick(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_sym = "G"
        self.stick_sym = "S"

        self.symbols_types = {
            self.gripper_sym: constants.type_gripper,
            self.stick_sym: constants.type_stick,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_sym, self.stick_sym)
        ]

        self.add_effects = [
            pred.HandEmpty(self.gripper_sym),
            pred.IsFree(self.stick_sym)
        ]

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        transientStep = 0.01
        sym2frame = _get_sym2frame(self.symbols, frames)

        stick = sym2frame[self.stick_sym]

        # get stick length
        stick_frame = C.getFrame(stick)
        stick_length = stick_frame.getSize()[1]

        grab_pos = (0, -stick_length / 3, 0)

        gripper = sym2frame[self.gripper_sym]
        stick = sym2frame[self.stick_sym]
        stick_handle = stick + "Handle"

        open_stick = ry.CtrlSet()
        open_stick.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper, stick), False)

        controllers = [
            ("open_stick", open_stick),
        ]

        return add_action_name(self.name, controllers)


class HandOver(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_give_sym = "G1"
        self.gripper_take_sym = "G2"
        self.block_sym = "B"

        self.symbols_types = {
            self.gripper_give_sym: constants.type_gripper,
            self.gripper_take_sym: constants.type_gripper,
            self.block_sym: constants.type_block,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_give_sym, self.block_sym),
            pred.HandEmpty(self.gripper_take_sym),
        ]

        self.add_effects = [
            pred.InHand(self.gripper_take_sym, self.block_sym),
            pred.HandEmpty(self.gripper_give_sym),
        ]
        # self.delete_effects = []

        self.delete_effects = self.preconditions

    def get_grounded_control_set(self, C, frames):
        hand_over_pos_1 = [0, 0.0, 1.3]
        # hand_over_pos_2
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper_give = sym2frame[self.gripper_give_sym]

        gripper_take = sym2frame[self.gripper_take_sym]

        block = sym2frame[self.block_sym]

        align_1 = ry.CtrlSet()

        align_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_give, block), True)

        # transient objectives
        align_1.addObjective(
            C.feature(ry.FS.position, [block], [1e1], hand_over_pos_1),
            ry.OT.sos, transient_step)
        # align_1.addObjective(
        #     C.feature(ry.FS.positionDiff, [gripper_give, block], [1e0]),
        #     ry.OT.eq, -1)

        direction = 1 if gripper_give == "r_gripper" else -1

        hand_over_pos_align_take = list(hand_over_pos_1)
        hand_over_pos_align_take[0] -= direction*0.1

        align_1.addObjective(
            C.feature(ry.FS.scalarProductXZ, ["world", gripper_give], [1e1], [direction]),
            ry.OT.sos, transient_step)

        align_1.addObjective(
            C.feature(ry.FS.scalarProductXZ, ["world", gripper_take], [1e1], [-direction]),
            ry.OT.sos, transient_step)

        align_1.addObjective(
            C.feature(ry.FS.position, [gripper_take], [1e1], hand_over_pos_align_take),
            ry.OT.sos, transient_step)

        cage = ry.CtrlSet()
        cage.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_give, block), True)

        cage.addObjective(
            C.feature(ry.FS.scalarProductXZ, ["world", gripper_give], [1e0], [direction]),
            ry.OT.eq, -1)

        # cage.addObjective(
        #     C.feature(ry.FS.scalarProductXZ, ["world", gripper_give], [1e1], [direction]),
        #     ry.OT.eq, -1)
        cage.addObjective(
            C.feature(ry.FS.position, [block], [1e0], hand_over_pos_1),
            ry.OT.eq, -1)
        cage.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_take, block], [1e3]),
            ry.OT.sos, transient_step / 10)

        cage.addObjective(
            C.feature(ry.FS.scalarProductZZ, [gripper_give, gripper_take], [1e2], [-1]),
            ry.OT.sos, transient_step)
        cage.addObjective(
            C.feature(ry.FS.scalarProductXX, [gripper_give, gripper_take], [1e2]),
            ry.OT.sos, transient_step*2)
        # cage.addObjective(
        #     C.feature(ry.FS.scalarProductYY, [gripper_take, "world"], [1e1]),
        #     ry.OT.sos, speed)

        hand_over_1 = ry.CtrlSet()
        hand_over_2 = ry.CtrlSet()

        # hand_over_1.addObjective(
        #     C.feature(ry.FS.positionRel, [block, gripper_give], [1e0]),
        #     ry.OT.eq, -1)

        hand_over_1.addObjective(
            C.feature(ry.FS.scalarProductZZ, [gripper_give, gripper_take], [1e0], [-1]),
            ry.OT.eq, -1)

        hand_over_1.addObjective(
            C.feature(ry.FS.scalarProductXZ, ["world", gripper_give], [1e1], [direction]),
            ry.OT.sos, transient_step)

        hand_over_1.addObjective(
            C.feature(ry.FS.position, [block], [1e0], hand_over_pos_1),
            ry.OT.eq, -1)

        # hand_over_2.addObjective(
        #     C.feature(ry.FS.scalarProductXZ, ["world", gripper_take], [1e1], [-1*direction]),
        #     ry.OT.sos, speed)
        # hand_over_2.addObjective(
        #     C.feature(ry.FS.scalarProductXY, ["world", gripper_take], [1e0], [direction]),
        #     ry.OT.eq, -1)

        for hand_o in [hand_over_1, hand_over_2]:
            hand_o.addObjective(
                C.feature(ry.FS.positionRel, [gripper_take, block], [1e1]),
                ry.OT.eq, -1)

            # hand_o.addObjective(
            #     C.feature(ry.FS.scalarProductXY, ["world", gripper_take], [1e1], [direction]),
            #     ry.OT.eq, -1)

        # condition, nothing is in hand of gripper
        hand_over_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_give, block), True)
        hand_over_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_take, block), False)
        # hand_over_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_take, block), False)

        # hand_over_1.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_take, block), True)
        # hand_over_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_take, block), False)

        # condition, nothing is in hand of gripper
        # hand_over_2.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_give, block), False)
        # hand_over_2.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_give, block), True)
        #lets hope this is executed last...
        hand_over_1.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_give, block), False)
        # hand_over_2.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_take, block), True)

        controllers = [
            ("align_1", align_1),
            # ("align_2", align_2),
            ("cage", cage),
            ("hand_over_1", hand_over_1),
            # ("hand_over_2", hand_over_2)
        ]

        return add_action_name(self.name, controllers)


class OpenBottle(BaseAction):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.gripper_1_sym = "G1"
        self.gripper_2_sym = "G2"
        self.bottle_sym = "B"

        self.symbols_types = {
            self.gripper_1_sym: constants.type_gripper,
            self.gripper_2_sym: constants.type_gripper,
            self.bottle_sym: constants.type_bottle,
        }

        self.symbols = self.symbols_types.keys()

        self.preconditions = [
            pred.InHand(self.gripper_1_sym, self.bottle_sym),
            pred.HandEmpty(self.gripper_2_sym),
        ]

        self.add_effects = [
            pred.BottleOpen(self.bottle_sym),
        ]
        # self.delete_effects = []

        self.delete_effects = [self.preconditions[1]]

    def get_grounded_control_set(self, C, frames):
        hand_over_pos_1 = [0, 0.0, 1.2]
        sym2frame = _get_sym2frame(self.symbols, frames)

        gripper_1 = sym2frame[self.gripper_1_sym]
        gripper_1_center = gripper_1 + "Center"

        gripper_2 = sym2frame[self.gripper_2_sym]
        gripper_2_center = gripper_2 + "Center"

        bottle = sym2frame[self.bottle_sym]
        cap = "cap"

        # align gripper 2 with z axis of bottle
        align_1 = ry.CtrlSet()
        align_1.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, bottle), True)

        # align_1.addObjective(
        #     C.feature(ry.FS.position, [block], [1e2], hand_over_pos_1),
        #     ry.OT.sos, 0.005)
        align_1.addObjective(
            C.feature(ry.FS.vectorZDiff, [bottle, gripper_2], [1e2]),
            ry.OT.sos, 0.01)
        align_1.addObjective(
            C.feature(ry.FS.positionRel, [gripper_2, cap], [1e2], [0, 0, 0.1]),
            ry.OT.sos, 0.02)

        cage = ry.CtrlSet()
        cage.addObjective(
            C.feature(ry.FS.vectorZDiff, [cap, gripper_2], [1e1]),
            ry.OT.eq, -1)
        cage.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_2_center, cap], [1e2]),
            ry.OT.sos, 0.005)
        cage.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, bottle), True)

        grab_cap = ry.CtrlSet()
        grab_cap.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_2_center, cap], [1e1]),
            ry.OT.eq, -1)
        grab_cap.addObjective(
            C.feature(ry.FS.vectorZDiff, [cap, gripper_2], [1e1]),
            ry.OT.eq, -1)
        grab_cap.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, bottle), True)
        grab_cap.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_2, cap), False)

        # turn cap
        turn_cap = ry.CtrlSet()
        turn_cap.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, bottle), True)
        turn_cap.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_2, cap), True)

        # turn_cap.addObjective(
        #     C.feature(ry.FS.vectorZDiff, [bottle, cap], [1e0]),
        #     ry.OT.eq, -1)
        #
        # turn_cap.addObjective(
        #     C.feature(ry.FS.quaternionDiff, [bottle, cap], [1e2], [0,0,0,1]),
        #     ry.OT.sos, 0.005)

        # grab_cap.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_2, cap), False)
        # align_2.addObjective(
        #     C.feature(ry.FS.scalarProductZZ, [gripper_1, gripper_2], [1e2], [-1]),
        #     ry.OT.sos, 0.01)
        # align_2.addObjective(
        #     C.feature(ry.FS.scalarProductXX, [gripper_1, gripper_2], [1e2]),
        #     ry.OT.sos, 0.01)
        # align_2.addObjective(
        #     C.feature(ry.FS.position, [block], [1e1], hand_over_pos_1),
        #     ry.OT.eq, -1)
        #
        # cage = ry.CtrlSet()

        # cage.addObjective(
        #     C.feature(ry.FS.distance, [gripper_1, gripper_2], [1e1], [0.25]),
        #     ry.OT.ineq, -1)
        # cage.addObjective(
        #     C.feature(ry.FS.position, [block], [1e1], hand_over_pos_1),
        #     ry.OT.eq, -1)
        # cage.addObjective(
        #     C.feature(ry.FS.scalarProductZZ, [gripper_1, gripper_2], [1e2], [-1]),
        #     ry.OT.eq, -1)
        # cage.addObjective(
        #     C.feature(ry.FS.positionDiff, [gripper_1_center, gripper_2_center], [1e2]),
        #     ry.OT.sos, 0.005)

        hand_over = ry.CtrlSet()
        hand_over.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_1_center, bottle], [1e2]),
            ry.OT.eq, -1)
        hand_over.addObjective(
            C.feature(ry.FS.positionDiff, [gripper_2_center, bottle], [1e2]),
            ry.OT.eq, -1)
        # condition, nothing is in hand of gripper
        hand_over.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_1, bottle), False)
        hand_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_1, bottle), True)

        hand_over.addSymbolicCommand(ry.SC.OPEN_GRIPPER, (gripper_2, bottle), True)
        hand_over.addSymbolicCommand(ry.SC.CLOSE_GRIPPER, (gripper_2, bottle), False)

        controllers = [
            ("align_1", align_1),
            ("cage_cap", cage),
            ("grab_cap", grab_cap),
            ("turn_cap", turn_cap)
        ]

        return add_action_name(self.name, controllers)
