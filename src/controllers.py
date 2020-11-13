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

        # get symbols starting with A
        #symbol = [chr(x + ord('A')) for x in range(len(self.frames_symbol))]

        # list of tuples, with the frames and their symbols
        action_parameters = tuple(zip(self.frame_type, self.frames_symbol))

        # temporary dictionary we need to translate from frame to symbol
        #frame2symbol = dict((zip(self.frames_symbol, symbol)))

        preconditions = [(str(o.FS), *o.frames_symbol) for o in self.objectives if o.isImmediate()]

        effects = [(str(o.FS), *o.frames_symbol) for o in self.objectives if not o.isImmediate()]

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


class GraspBlock(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        self.frame_type = ["block", "gripper"]
        self.frames_symbol = ["B1", "G"]

        self.objectives.extend((
            Objective(
                FS=ry.FS.vectorZDiff,
                frames=self.frames_symbol,
                scale=[1e1],
                type=ry.OT.sos,
                transientStep=.005
            ),
            Objective(
                FS=ry.FS.distance,
                frames=self.frames_symbol,
                scale=[1e1],
                type=ry.OT.sos,
                transientStep=.005
            )
        ))
        # print(self.objectives)


class PlaceOn(BaseController):

    def __init__(self):
        super().__init__(__class__.__name__)

        # define symbols here
        self.frame_type = ["block", "gripper", "block"]
        self.frames_symbol = ["B1", "G", "B2"]

        self.objectives.extend((
            Objective(
                FS=ry.FS.distance,
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                scale=[1e1],
                type=ry.OT.ineq,
                transientStep=.005
            ),
            Objective(
                ry.FS.vectorZDiff,
                frames=[self.frames_symbol[0], self.frames_symbol[1]],
                scale=[1e1],
                type=ry.OT.ineq,
                transientStep=.005

            ),
            Objective(
                ry.FS.standingAbove,
                frames=[self.frames_symbol[0], self.frames_symbol[2]],
                scale=[1e1],
                type=ry.OT.sos,
                transientStep=.005
            )
        ))


class Objective:

    def __init__(self, FS, frames, scale, type, target=0, transientStep=-1.0):
        self.FS = FS
        self.frames_symbol = frames
        self.scale = scale
        self.type = type
        self.transientStep = transientStep

    def isImmediate(self):
        return self.type is ry.OT.eq or self.type is ry.OT.ineq

    def isTransient(self):
        return not self.isImmediate()

    def objective2symbol(self):
        return str(self.FS)[3:] + "/" + "/".join(self.frames_symbol)

    def groundObjective(self, frames_real):
        """
        Create an instance of this objective
        """
