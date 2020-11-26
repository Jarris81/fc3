import libry as ry
from pyddl import Action, neg
from objective import Objective
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

    def get_simple_action(self):

        # This needs to be implemented for each controller
        print(f"This function needs to be implemented! {self.__name__}")

    def get_action_pyddl(self, all_objects):

        # all_obj_grounded = set([x for x in it.product(*all_objects.values())])
        # list of tuple of all objects of all

        all_types_count = {obj_type: len(all_objects[obj_type]) for obj_type in all_objects.keys()}
        self.set_frame_type_count()

        delete_types_count = {x: all_types_count[x] - self.frame_type_count[x] if x in self.frame_type_count \
            else all_types_count[x] for x in all_objects.keys()}

        # list of tuples, with the frames and their symbols
        action_parameters = list(zip(self.frame_type, self.frames_symbol))

        # delete_predicates = all_action_parameters.difference(all_action_parameters)
        #
        # print("all_action_parameters", all_action_parameters)
        # print("action_parameters", action_parameters)
        #print("delete: ", delete_types_count)

        delete_parameters = []

        counter = 0
        for x in delete_types_count:
            for i in range(delete_types_count[x]):
                temp = (x, chr(ord("Z")-counter))
                action_parameters.append(temp)
                delete_parameters.append(temp)
                counter += 1

        print("action_parameters", action_parameters)


        type2sym = {
            str(ry.OT.eq): "=",
            str(ry.OT.ineq): "<=",
            str(ry.OT.sos): "=="
        }

        preconditions = []
        effects = []

        for o in self.objectives:
            # special case: when we are focusing a block, we need to unset focus on other blocks
            # otherwise we get a weird behavior (double approach)

            if o.FS == "focus" and o.is_transient():
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

    def activate_controller(self, C, frames_realized, scales):

        symbol2real = dict(zip(self.frames_symbol, frames_realized))

        for objective in self.objectives:
            frames_real_objective = [symbol2real[frame] for frame in objective.frames_symbol]
            objective.groundObjective(frames_real_objective)


class Approach(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = ["block", "gripper"]
        self.frames_symbol = ["B1", "G"]

        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS=ry.FS.distance,
                frames=self.frames_symbol,
                target=[1],
                OT_type=ry.OT.sos,
                scale=[1],
                transientStep=.005
            ),
            Objective(
                FS=ry.FS.vectorZDiff,
                frames=self.frames_symbol,
                target=[1] * 3,
                OT_type=ry.OT.sos,
                scale=[1] * 3,
                transientStep=.005
            ),
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.sos,
            )
        ))


class CloseGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = ["block", "gripper"]
        self.frames_symbol = ["B1", "G"]
        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS=ry.FS.distance,
                frames=self.frames_symbol,
                target=[1],
                OT_type=ry.OT.ineq,
                scale=[1],
                transientStep=.005
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
        ))


class PlaceOn(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        # define symbols here
        self.frame_type = ["block", "gripper", "block"]
        self.frames_symbol = ["B1", "G", "B2"]

        self.target = [self.frames_symbol[0]]

        self.objectives.extend((
            Objective(
                FS=ry.FS.vectorZDiff,
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                OT_type=ry.OT.ineq,
                target=[10] * 3,
                scale=[1e0] * 3,
            ),
            Objective(
                FS="grasping",
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                OT_type=ry.OT.eq,
                transientStep=.005
            ),
            Objective(
                FS=ry.FS.distance,
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                OT_type=ry.OT.ineq,
                target=[1],
                scale=[1e0],
            ),
            Objective(
                FS=ry.FS.standingAbove,
                frames=[self.frames_symbol[0], self.frames_symbol[2]],
                OT_type=ry.OT.sos,
                target=[1, 1, 1, 1],
                scale=[1e0],
                transientStep=.005
            ),
            Objective(
                FS="focus",
                frames=self.target,
                OT_type=ry.OT.eq,
            ),
            Objective(
                FS="un_focus",
                frames=self.target,
                OT_type=ry.OT.sos)
        ))
