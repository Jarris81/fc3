import libry as ry
import numpy as np


class SimpleExecutor:

    def __init__(self, C):
        self.C = C

        # home controller, used when robot is done with plan
        self.home_controller = ry.CtrlSet()
        self.q_home = C.getJointState()
        self.home_controller.addObjective(
            C.feature(ry.FS.qItself, [], [1e2], self.q_home),
            ry.OT.sos, 0.05
        )

        self.q = None
        self.q_old = None

        # precision needed to initiate a controller
        self.eqPrecision = 1e-2

        self.home_controller.add_qControlObjective(2, 1e-5 * np.sqrt(0.01),
                                                   self.C)  # TODO this will make some actions unfeasible (PlaceSide)
        self.home_controller.add_qControlObjective(1, 1e-3 * np.sqrt(0.01), self.C)

        self.log = self.log_default

    def move_home(self):
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        ctrl.set(self.home_controller)
        ctrl.update(self.C.getJointState(), [], self.C)
        return ctrl.solve(self.C)

    def is_home(self):
        # ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        # TODO: not working, convergence should be
        # return self.home_controller.isConverged(ctrl, self.eqPrecision)
        q_home = self.q_home
        q_cur = self.C.getJointState()
        is_home = np.allclose(self.C.getJointState(), self.q_home, self.eqPrecision, self.eqPrecision)
        return is_home

    def is_goal_fulfilled(self):
        ctrl = ry.CtrlSolver(self.C, 0.1, 2)
        return self.goal_controller.canBeInitiated(ctrl, self.eqPrecision)

    def is_no_plan_feasible(self):
        return self.no_plan_feasible

    def get_gripper_action(self):

        return self.gripper_action

    def step(self, t, tau):

        pass
