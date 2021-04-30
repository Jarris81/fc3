import util.domain_tower as dt
import libry as ry
import numpy as np
import itertools


class BasePredicate:

    def __init__(self):
        self.name = self.__class__.__name__
        self.sym2frame = None

    def ground_predicate(self, **sym2frames):
        self.sym2frame = sym2frames

    def is_feasible(self, C, *objects):
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

    def features(self, C):
        if self.sym2frame is None:
            print(f"{self.name} has not been grounded!")
            return None
        return []

    def _all_features_feasible(self, C):

        errors = [feat.eval(C)[0] for feat in self.features(C)]

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

    def features(self, C):
        super().features(C)

        block = self.sym2frame[self.block_sym]
        gripper = self.sym2frame[self.gripper_sym]
        gripper_center = gripper + "Center"
        return [
            C.feature(ry.FS.distance, [block, gripper_center], [1e1]),
        ]

    def is_feasible(self, C, *objects):
        # call super to make sure predicate has been has grounded
        super().is_feasible(C)

        # get names
        gripper_name = self.sym2frame[self.gripper_sym]
        block_name = self.sym2frame[self.block_sym]

        # check if distance is small enough and if gripper is parent of block
        block_frame = C.frame(block_name)

        # True until proven otherwise
        if "parent" in block_frame.info() and not block_frame.info()["parent"] == gripper_name:
            return False

        # check if all the features are ok
        return self._all_features_feasible(C)


class HandFree(BasePredicate):

    def __init__(self, gripper_sym):
        super().__init__()
        self.gripper_sym = gripper_sym

    def get_predicate(self):
        return self.name, self.gripper_sym

    def is_feasible(self, C, *objects):
        # need to check if no object has the gripper as parent
        for frameName in objects:
            frame = C.frame(frameName)
            if "parent" in frame.info() and frame.info()["parent"] == self.sym2frame[self.gripper_sym]:
                print(frameName)
                return False
        return True


class BlockFree(BasePredicate):

    def __init__(self, block_sym):
        super().__init__()
        self.block_sym = block_sym

    def get_predicate(self):

        return self.name, self.block_sym

    def is_feasible(self, C, *objects):
        # build for each pair of block the reverse predicate, and check if they are all true
        objects = set(objects)
        block = self.sym2frame[self.block_sym]

        objects.remove(block)

        all_pairs = list(itertools.product([block], objects))

        opposite_predicates = [BlockOnBlock("B", "B2") for _ in range(len(all_pairs))]
        [pred.ground_predicate(B=x[1], B2=x[0]) for pred, x in zip(opposite_predicates, all_pairs)]

        return not any([x.is_feasible(C, objects) for x in opposite_predicates])


class BlockOnBlock(BasePredicate):

    def __init__(self, block_sym, block_placed_on_sym):
        super().__init__()
        self.block_sym = block_sym
        self.block_placed_on_sym = block_placed_on_sym

    def get_predicate(self):
        return self.name, self.block_sym, self.block_placed_on_sym

    def features(self, C):

        super().features(C)

        block = self.sym2frame[self.block_sym]
        block_placed_on = self.sym2frame[self.block_placed_on_sym]

        return [
            C.feature(ry.FS.positionRel, [block, block_placed_on], [1e1], [0, 0, 0.105]),
            C.feature(ry.FS.scalarProductZZ, [block, block_placed_on], [1e1], [1]),
        ]

    def is_feasible(self, C, *objects):
        # call super to make sure predicate has been grounded
        super().is_feasible(C)

        return self._all_features_feasible(C)



