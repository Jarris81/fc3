import util.constants as dt
import libry as ry
import numpy as np
import itertools


class BasePredicate:

    def __init__(self):
        self.name = self.__class__.__name__
        self.sym2frame = None

    def ground_predicate(self, **sym2frames):
        self.sym2frame = sym2frames

    def is_feasible(self, C, all_frames):
        if self.sym2frame is None:
            print(f"{self.name} has not been grounded!")
            return None
        return False

    def get_predicate(self):
        print(f"This function needs to be implemented for class {self.name}")
        return []

    def get_grounded_predicate(self):
        if self.sym2frame is None:
            print(f"{self.name} has not been grounded!")
            return None

        predicate = self.get_predicate()
        return tuple([predicate[0]] + [self.sym2frame[x] for x in predicate[1:]])

    def features(self, C, all_frames):
        if self.sym2frame is None:
            print(f"{self.name} has not been grounded!")
            return None
        return []

    def symbolic_commands(self, C, all_frames):
        if self.sym2frame is None:
            print(f"{self.name} has not been grounded!")
            return None
        return []

    def _all_features_feasible(self, C, all_frames):

        errors = [feat.eval(C)[0] for feat in self.features(C, all_frames)]

        print(errors)

        return all([np.sqrt(np.dot(error, error)) < 0.1 for error in errors])


class CollisionFree(BasePredicate):
    """
    Special predicate, since not applied to only one frame, but all
    """

    def __init__(self):
        super().__init__()

    def get_predicate(self):
        return self.name,

    def ground_predicate(self, **sym2frames):
        self.sym2frame = sym2frames

    def is_feasible(self, C, *objects):
        C.computeCollisions()
        collisions = C.getCollisions()
        return all(col[2] > 0 for col in collisions)


class InHand(BasePredicate):

    def __init__(self, gripper_sym, block_sym):
        super().__init__()
        self.gripper_sym = gripper_sym
        self.block_sym = block_sym

    def get_predicate(self):
        return self.name, self.gripper_sym, self.block_sym

    def features(self, C, all_frames):
        super().features(C, all_frames)

        block = self.sym2frame[self.block_sym]
        gripper = self.sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        gripper_preGrasp = gripper + "Pregrasp"
        return [
            C.feature(ry.FS.insideBox, [block, gripper_preGrasp], [1e1]),
        ]
        # return [
        #     C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
        # ]

    def is_feasible(self, C, all_frames):
        # call super to make sure predicate has been has grounded
        super().is_feasible(C, all_frames)

        # get names
        gripper_name = self.sym2frame[self.gripper_sym]
        block_name = self.sym2frame[self.block_sym]

        # check if distance is small enough and if gripper is parent of block
        block_frame = C.frame(block_name)

        # True until proven otherwise
        if "parent" in block_frame.info() and not block_frame.info()["parent"] == gripper_name:
            return False

        # check if all the features are ok
        return self._all_features_feasible(C, all_frames)

    def symbolic_commands(self, C, all_frames):
        commands = [(ry.SC.CLOSE_GRIPPER, (self.sym2frame[self.gripper_sym], self.sym2frame[self.block_sym]), True)]
        return commands


class HandEmpty(BasePredicate):

    def __init__(self, gripper_sym):
        super().__init__()
        self.gripper_sym = gripper_sym

    def get_predicate(self):
        return self.name, self.gripper_sym

    def features(self, C, all_frames):
        all_frames = set(all_frames)
        rel_frames = {self.sym2frame[self.gripper_sym]}
        other_frames = all_frames.difference(rel_frames)
        features = []
        gripper_frame = self.sym2frame[self.gripper_sym]
        gripper_pre_grasp_frame = gripper_frame + "Pregrasp"
        for frame in other_frames:
            features.append(
                C.feature(ry.FS.pairCollision_negScalar, [frame, gripper_pre_grasp_frame], [1e1])
            )
        return []
        #return features

    def symbolic_commands(self, C, all_frames):
        commands = []
        all_frames = set(all_frames)
        rel_frames = {self.sym2frame[self.gripper_sym]}
        other_frames = all_frames.difference(rel_frames)
        for object in other_frames:
            commands.append((ry.SC.OPEN_GRIPPER, (self.sym2frame[self.gripper_sym], object), True))
        return commands


class BlockFree(BasePredicate):

    def __init__(self, block_sym):
        super().__init__()
        self.block_sym = block_sym
        self.rel_symbols = [self.block_sym]

    def get_predicate(self):

        return self.name, self.block_sym

    def features(self, C, all_frames):

        features = []
        block_frame = self.sym2frame[self.block_sym]

        all_frames = set(all_frames)
        rel_frames = {block_frame}
        other_frames = all_frames.difference(rel_frames)
        block_place_box_frame = block_frame + "_place_box"
        for frame in other_frames:
            if not "gripper" in frame:
                features.append(
                    C.feature(ry.FS.pairCollision_negScalar, [frame, block_place_box_frame], [1e0])
                )

        return features


class BlockOnBlock(BasePredicate):

    def __init__(self, block_sym, block_placed_on_sym):
        super().__init__()
        self.block_sym = block_sym
        self.block_placed_on_sym = block_placed_on_sym

    def get_predicate(self):
        return self.name, self.block_sym, self.block_placed_on_sym

    def features(self, C, all_frames):

        super().features(C, all_frames)

        block = self.sym2frame[self.block_sym]
        block_placed_on = self.sym2frame[self.block_placed_on_sym]

        return [
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e1], [0, 0, 0.105]),
            C.feature(ry.FS.scalarProductZZ, [block, block_placed_on], [1e1], [1]),
        ]

    def is_feasible(self, C, all_frames):
        # call super to make sure predicate has been grounded
        super().is_feasible(C, all_frames)

        return self._all_features_feasible(C, all_frames)


class BlockAtGoal(BasePredicate):
    def __init__(self, block_sym):
        super().__init__()
        self.block_sym = block_sym

    def get_predicate(self):
        return self.name, self.block_sym


class BlockIsClose(BasePredicate):

    def __init__(self, block_sym):
        super().__init__()
        self.block_sym = block_sym

    def get_predicate(self):
        return self.name, self.block_sym

