import util.constants as constants

class InterferenceTemplate:

    def __init__(self, start, end, max_count, description):
        self.start = start
        self.end = end
        self.max_count = max_count
        self.count = 0
        self.description = description

    def do_interference(self, C, t):
        if self.count >= self.max_count or not self.start < t < self.end:
            return False

        self.count += 1

        return True

    def reset(self):
        self.count = 0


class NoInterference(InterferenceTemplate):

    def __init__(self):
        super().__init__(0, 0, 0, "None")


class ResetPosition(InterferenceTemplate):

    def __init__(self, start, end, frame_name, pos, name, max_count=1, ):
        super().__init__(start, end, max_count, name)
        self.frame_name = frame_name
        self.pos = pos

    def do_interference(self, C, t):
        if super().do_interference(C, t):
            box = C.frame(self.frame_name)
            box.setPosition(self.pos)
            box.setQuaternion([1, 0, 0, 0])


def get_tower_interferences():
    # first is always no interference
    interference_list = [NoInterference()]
    infeasible_pos_b1 = (0.6, 0.6, 0.68)

    ori_pos_b2 = 0.4, 0.3, constants.table_height + 0.03

    interference_list.extend((
        # 1: b2 is moved while robot is moving to it
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
        # 2: b2 is moved while robot is grabing it
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot grabs it"),
        # 3: b2 is knocked of tower while gripper is moving to b1
        ResetPosition(11, 13, "b2", ori_pos_b2, "b2 falls of tower while robot moves to b1"),
        # 4: b1 is moved out of reach
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 is moved out of reach"),
        # 5: b2 is knocked of tower while gripper is holding b1
        ResetPosition(150, 262, "b2", ori_pos_b2, "b2 falls of tower while robot holds b1"),
        # 6: b2 is placed on b3 (Helping Hand)
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is placed on b3 (helping Hand)"),
    ))

    return interference_list


def get_stick_interferences():
    # first is always no interference
    interference_list = [NoInterference()]
    infeasible_pos_b1 = (0.6, 0.6, 0.68)

    ori_pos_b2 = 0.4, 0.3

    interference_list.extend((
        # 1
        ResetPosition(190, 192, "b1", ori_pos_b2, "b1 is moved while robot moves to it or is grabbing"),
        # 2
        ResetPosition(150, 262, "b2", ori_pos_b2, "b1 is moved out of reach"),
        # 3
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 is moved out of reach for robot, but reachable with the stick"),
        # 4
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 and stick are unreachable"),
    ))

    return interference_list


def get_hand_over_interferences():
    # first is always no interference
    interference_list = []
    infeasible_pos_b1 = (0.6, 0.6, 0.68)

    ori_pos_b2 = 0.4, 0.3

    interference_list.extend((
        # 0
        ResetPosition(190, 192, "b1", ori_pos_b2, "b1 goal is left side, starts on left (no interference)"),
        # 1
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 goal is left side, starts on left, "
                                                       "interference is moving to the far right side"),
        # 2
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 goal is left side, starts on left,"
                                                       "interference is moving to the middle"),
        # 3
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 goal is left side, starts on right, (no interference)"),
        # 4
        ResetPosition(0,0, "b1", infeasible_pos_b1, "b1 goal is left side, start on right, moved to left"),
        # 5
        ResetPosition(0,0, "b1", infeasible_pos_b1, "b1 goal is left side, starts on right, moved to middle "
                                                    "(reachable by both)")
    ))

    return interference_list