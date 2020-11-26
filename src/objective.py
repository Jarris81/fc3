import libry as ry
from pyddl import Action, neg


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

    def get_pyddl_description(self, type2sym, target=None):

        predicates = []
        if self.FS == "grasping":
            predicates.extend([(self.FS, *self.frames_symbol)])

        if self.FS == "not_grasping":
            predicates.extend([neg(("grasping", *self.frames_symbol))])

        if self.FS == "focus":
            return [(self.FS, *self.frames_symbol)]

        if self.FS == "not_focus":
            return [neg(("focus", *self.frames_symbol))]

        if self.FS == "gripper_free":
            return [(self.FS, *self.frames_symbol)]

        if self.FS == "cool":
            return [(self.FS, *self.frames_symbol)]

        if self.FS == "not_gripper_free":
            return [neg(("gripper_free", *self.frames_symbol))]

        if self.FS.__class__ == ry.FS:
            predicates.extend((type2sym[str(self.type)], (f"{self.FS}_{i}", *self.frames_symbol), int(self.target[i]))
                              for i in range(len(self.target)))

        # if target is not None:
        #     predicates.extend([("target", target)])

        return predicates
