import libry as ry
from pyddl import Action
import array


class BaseController:

    def __init__(self, name):
        self.name = name
        self.objectives = []
        self.frame_type = []

        self.frames_symbol = []

    def getAllObjectives(self):

        return self.objectives

    def getImmediateObjectives(self):

        return [obj for obj in self.o]

    def get_description(self):

        for objective in self.objectives:
            print(objective.objective2symbol())

    def get_action_pyddl(self):

        # list of tuples, with the frames and their symbols
        action_parameters = tuple(zip(self.frame_type, self.frames_symbol))

        type2sym = {
            str(ry.OT.eq): "=",
            str(ry.OT.ineq): "<=",
            str(ry.OT.sos): "=="
        }

        preconditions = []
        effects = []

        for o in self.objectives:
            if o.is_immediate():
                preconditions.extend(o.get_pyddl_description(type2sym))
            elif o.is_transient():
                effects.extend(o.get_pyddl_description(type2sym))
            else:
                print("Objectives should be immediate or transient only!")

        return Action(
            name=self.name,
            parameters=action_parameters,
            preconditions=preconditions,
            effects=effects
        )

    def activate_controller(self, C, frames_realized, scales):

        symbol2real = dict(zip(self.frames_symbol, frames_realized))

        for objective in self.objectives:
            frames_real_objective = [symbol2real[frame] for frame in objective.frames_symbol]
            objective.groundObjective(frames_real_objective)


class CloseGripper(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = ["block", "gripper"]
        self.frames_symbol = ["B1", "G"]

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
                FS="grasping",
                frames=self.frames_symbol,
                OT_type=ry.OT.sos,
                transientStep=.005
            ),
        ))


class GraspBlock(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = ["block", "gripper"]
        self.frames_symbol = ["B1", "G"]

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
            )
        ))


class PlaceOn(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        # define symbols here
        self.frame_type = ["block", "gripper", "block"]
        self.frames_symbol = ["B1", "G", "B2"]

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
            )
        ))


class Objective:

    def __init__(self, FS, frames, OT_type, target=None, scale=None, transientStep=-1.0):
        self.FS = FS
        self.frames_symbol = frames
        self.target = target
        self.scale = scale
        self.type = OT_type
        self.transientStep = transientStep

    def is_immediate(self):
        return self.type is ry.OT.eq or self.type is ry.OT.ineq

    def is_transient(self):
        return not self.is_immediate()

    def objective2symbol(self):
        return str(self.FS)[3:] + "/" + "/".join(self.frames_symbol)

    def groundObjective(self, frames_real):
        """
        Create an instance of this objective
        """

    def get_pyddl_description(self, type2sym):

        if self.FS == "grasping":
            return [(self.FS, *self.frames_symbol)]

        return ((type2sym[str(self.type)], (f"{self.FS}_{i}", *self.frames_symbol), int(self.target[i]))
                for i in range(len(self.target)))

