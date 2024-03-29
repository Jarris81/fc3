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
        # b2 is moved while robot is moving to it
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
        # b2 is moved while robot is grabing it
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot grabs it"),
        # b2 is knocked of tower while gripper is moving to b1
        ResetPosition(11, 13, "b2", ori_pos_b2, "b2 falls of tower while robot moves to b1"),
        # b2 is knocked of tower while gripper is holding b1
        ResetPosition(150, 262, "b2", ori_pos_b2, "b2 falls of tower while robot holds b1"),
        # b1 is moved out of reach
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
        #  Robot is disturbed
        ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
    ))

    return interference_list


def get_stick_interferences():
    # first is always no interference
    interference_list = [NoInterference()]
    infeasible_pos_b1 = (0.6, 0.6, 0.68)

    ori_pos_b2 = 0.4, 0.3

    # interference_list.extend((
    #     # b2 is knocked of tower while gripper is moving to b1
    #     ResetPosition(190, 192, "b2", ori_pos_b2, "b2 falls of tower while robot moves to b1"),
    #     # b2 is knocked of tower while gripper is holding b1
    #     ResetPosition(150, 262, "b2", ori_pos_b2, "b2 falls of tower while robot holds b1"),
    #     # b1 is moved out of reach
    #     ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 is moved out of reach of robot"),
    #     #
    #     ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
    # ))

    return interference_list


def get_hand_over_interferences():
    # first is always no interference
    interference_list = [NoInterference()]
    infeasible_pos_b1 = (0.6, 0.6, 0.68)

    ori_pos_b2 = 0.4, 0.3

    # interference_list.extend((
    #     # b2 is knocked of tower while gripper is moving to b1
    #     ResetPosition(190, 192, "b2", ori_pos_b2, "b2 falls of tower while robot moves to b1"),
    #     # b2 is knocked of tower while gripper is holding b1
    #     ResetPosition(150, 262, "b2", ori_pos_b2, "b2 falls of tower while robot holds b1"),
    #     # b1 is moved out of reach
    #     ResetPosition(50, 52, "b1", infeasible_pos_b1, "b1 is moved out of reach of robot"),
    #     #
    #     ResetPosition(50, 52, "b1", infeasible_pos_b1, "b2 is moved while robot moves to it"),
    # ))

    return interference_list