import libry as ry
from pyddl import Action, neg
from objective import Objective
import util.domain_tower as dt
import itertools as it


class BaseController:

    def __init__(self, name):
        self.name = name
        self.objectives = []
        self.frame_type = []
        self.target = None

        self.frames_symbol = []
        self.frame_type_count = {}

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

    def get_action_pyddl(self, all_objects):

        # all_obj_grounded = set([x for x in it.product(*all_objects.values())])
        # list of tuple of all objects of all

        action_parameters = list(zip(self.frame_type, self.frames_symbol))

        type2sym = {
            str(ry.OT.eq): "=",
            str(ry.OT.ineq): "<=",
            str(ry.OT.sos): "=="
        }

        preconditions = []
        effects = []

        for o in self.objectives:

            if o.FS == "focus" and o.is_transient():
                # special case: when we are focusing a block, we need to unset focus on other blocks
                # otherwise we get a weird behavior (double approach)

                all_types_count = {obj_type: len(all_objects[obj_type]) for obj_type in all_objects.keys()}
                self.set_frame_type_count()

                delete_types_count = {x: all_types_count[x] - self.frame_type_count[x] if x in self.frame_type_count \
                    else all_types_count[x] for x in all_objects.keys()}

                # list of tuples, with the frames and their symbols
                action_parameters = list(zip(self.frame_type, self.frames_symbol))

                delete_parameters = []

                counter = 0
                for x in delete_types_count:
                    for i in range(delete_types_count[x]):
                        temp = (x, chr(ord("Z") - counter))
                        action_parameters.append(temp)
                        delete_parameters.append(temp)
                        counter += 1

                for x in delete_parameters:
                    #print("setting unfocus on ", x[1])
                    effects.append(neg(("focus", x[1])))

            if o.is_immediate():
                preconditions.extend(o.get_pyddl_description(type2sym, self.target))
            elif o.is_transient():
                effects.extend(o.get_pyddl_description(type2sym, self.target))

            else:
                print("Objectives should be immediate or transient only!")

        # look at other objects in the domain, unset their focus
        # effects.extend([("is_focus", *self.frames_symbol)])
        return Action(
            name=self.name,
            parameters=action_parameters,
            preconditions=preconditions,
            effects=effects,
            unique=True
        )

    def get_grounded_control_set(self, C, frames_realized):

        ctrl_set = ry.CtrlSet()
        symbol2real = dict(zip(self.frames_symbol, frames_realized))

        for o in self.objectives:
            real_frames = [symbol2real[x] for x in o.frames_symbol]
            if o.FS.__class__ == ry.FS:
                ctrl_set.addObjective(C.feature(o.FS, real_frames, o.scale, o.target), o.type, o.transientStep)

            elif o.FS == "grasping":
                grasp_command = [dt.SC_close_gripper, *real_frames[::-1]]  # hack, command is other way around
                is_condition = not o.is_transient()
                ctrl_set.addSymbolicCommand(grasp_command, is_condition)

            elif o.FS == "not_grasping":
                command = [dt.SC_open_gripper, *real_frames[::-1]]  # hack, command is other way around
                is_condition = not o.is_transient
                ctrl_set.addSymbolicCommand(command, is_condition)
            elif o.FS == "focus" or o.FS == "gripper_free" or o.FS == "not_gripper_free":
                # dont care about these, only symbolic
                continue
            else:
                print("objective not specified")
                print(o.FS)
                assert False

        return ctrl_set

    def _get_unset_effects(self, predicate, all_objects):
        """
        Get the
        """

        all_types_count = {obj_type: len(all_objects[obj_type]) for obj_type in all_objects.keys()}
        self.set_frame_type_count()

        delete_types_count = {x: all_types_count[x] - self.frame_type_count[x] if x in self.frame_type_count \
            else all_types_count[x] for x in all_objects.keys()}

        # list of tuples, with the frames and their symbols
        action_parameters = list(zip(self.frame_type, self.frames_symbol))

        delete_parameters = []

        effects = []

        counter = 0
        for x in delete_types_count:
            for i in range(delete_types_count[x]):
                temp = (x, chr(ord("Z") - counter))
                action_parameters.append(temp)
                delete_parameters.append(temp)
                counter += 1

        for x in delete_parameters:
            # print("setting unfocus on ", x[1])
            effects.append(neg(("focus", x[1])))

        return effects


class CloseGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = [dt.type_block, dt.type_gripper]
        self.frames_symbol = ["B1", "G"]
        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS=ry.FS.distance,
                frames=self.frames_symbol,
                target=[0],
                OT_type=ry.OT.eq,
                scale=[1],
            ),
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.eq,
            ),
            Objective(
                FS="grasping",
                frames=self.frames_symbol,
                OT_type=ry.OT.sos,
            ),
            Objective(
                FS="gripper_free",
                frames=self.frames_symbol[1],
                OT_type=ry.OT.eq,
            ),
            Objective(
                FS="not_gripper_free",
                frames=self.frames_symbol[1],
                OT_type=ry.OT.sos,
            ),
        ))

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
                      ))


class OpenGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = [dt.type_block, dt.type_gripper]
        self.frames_symbol = ["B1", "G"]
        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.eq,
            ),
            Objective(
                FS="grasping",
                frames=self.frames_symbol,
                OT_type=ry.OT.eq,
            ),
            Objective(
                FS="not_grasping",
                frames=self.frames_symbol,
                OT_type=ry.OT.sos,
            ),
            Objective(
                FS="gripper_free",
                frames=self.frames_symbol[1],
                OT_type=ry.OT.sos,
            ),
        ))

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
                      ))


class Approach(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = [dt.type_block, dt.type_gripper]
        self.frames_symbol = ["B1", "G"]

        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS=ry.FS.distance,
                frames=self.frames_symbol,
                target=[0],
                OT_type=ry.OT.sos,
                scale=[1e1],
                transientStep=.005
            ),
            Objective(
                FS=ry.FS.vectorZDiff,
                frames=self.frames_symbol,
                target=[0] * 3,
                OT_type=ry.OT.sos,
                scale=[1e1] * 3,
                transientStep=.005
            ),
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.sos,
            )
        ))

    def get_simple_action(self, all_objects=None):
        # special case, where we need to unset focus on other objects

        unset_focus = self._get_unset_effects(dt.focus, all_objects)
        #print(unset_effects)

        return Action(self.name,
                      parameters=(
                          (dt.type_gripper, "g"),
                          (dt.type_block, "b")
                      ),
                      preconditions=(
                          (dt.hand_empty, "g"),
                      ),
                      effects=(
                          (dt.focus, "b"),
                          *unset_focus
                      ))


class PlaceOn(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        # define symbols here
        self.frame_type = [dt.type_block, dt.type_gripper, dt.type_block]
        self.frames_symbol = ["B1", "G", "B2"]

        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            # Objective(
            #     FS=ry.FS.vectorZDiff,
            #     frames=[self.frames_symbol[0], self.frames_symbol[1]],
            #     OT_type=ry.OT.ineq,
            #     target=[10] * 3,
            #     scale=[1e0] * 3,
            # ),
            Objective(
                FS="grasping",
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                OT_type=ry.OT.eq
            ),
            Objective(
                FS=ry.FS.positionRel,
                frames=[self.frames_symbol[2], self.frames_symbol[0]],
                OT_type=ry.OT.sos,
                target=[0]*3,
                scale=[1e1]*3,
            ),
            Objective(
                FS=ry.FS.scalarProductZZ,
                frames=[self.frames_symbol[2], self.frames_symbol[0]],
                OT_type=ry.OT.sos,
                target=[1],
                scale=[1e1],
                transientStep=.005
            ),
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.eq,
            ),
        ))

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
                      ))
