class InterferenceTemplate:

    def __init__(self, start, end, max_count):

        self.start = start
        self.end = end
        self.max_count = max_count
        self.count = 0

    def do_interference(self, C, t):

        if self.count >= self.max_count or not self.start < t < self.end:
            return False

        self.count += 1

        return True

    def reset(self):
        self.count = 0


class NoInterference(InterferenceTemplate):

    def __init__(self):
        super().__init__(0, 0, 0)


class ResetPosition(InterferenceTemplate):

    def __init__(self, start, end, frame_name, pos, max_count=1):
        super().__init__(start, end, max_count)
        self.frame_name = frame_name
        self.pos = pos

    def do_interference(self, C, t):
        if super().do_interference(C, t):

            box = C.frame(self.frame_name)
            box.setPosition(self.pos)
            box.setQuaternion([1, 0, 0, 0])
