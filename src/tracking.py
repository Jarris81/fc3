import motioncapture
import util.constants as constants

class Tracker:

    def __init__(self, C, frame_names, rate):

        self.C = C
        self.frame_names = set(frame_names)
        self.rate = rate

        self.mc = motioncapture.MotionCaptureOptitrack("130.149.82.29")
        print("connected to motion capture")

    def update(self, t):

        if not t % self.rate:

            self.mc.waitForNextFrame()

            # get intersection of frames
            caputured_frames = self.frame_names.intersection(self.mc.rigidBodies.keys())

            for frame_name, obj in self.mc.rigidBodies.items():
                if frame_name in caputured_frames:

                    frame = self.C.frame(frame_name)

                    # add constants to get better values
                    pos = list(obj.position)
                    pos[2] += constants.table_height
                    frame.setPosition(pos)

        else:
            return
        # for each frame,
        pass
    def _check_angular_velocity(self):
        pass


if __name__ == '__main__':

    pass