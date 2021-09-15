import motioncapture
import numpy as np
import util.constants as constants
import time
import util.setup_env as setup_env

###
# Constant Offsets for objects and on general
###
object_offsets = {
    "b1": np.array([0, 0, 0]),
    "b2": np.array([0, 0, 0]),
    "b3": np.array([0, 0, 0]),
    "stick": np.array([0, 0, 0]),
}


class Tracker:
    # TODO, make this run in own thread (either botop or here)

    def __init__(self, C, frame_names, rate):

        self.C = C
        self.frame_names = set(frame_names)
        self.rate = rate

        self.mc = motioncapture.MotionCaptureOptitrack("130.149.82.29")

        self.info = {frame: {"avg_pos": np.zeros(3), "count": 0} for frame in self.frame_names}

        self.avg_rate = 100

        self.last_update = -1000

        self.dummy_frame_name = "opti_dummy"
        self.C.addFrame("opti_dummy")

    def update(self, t):

        # if t-self.last_update > self.rate:
        if True:

            self.mc.waitForNextFrame()

            # get intersection of frames
            captured_frames = self.frame_names.intersection(self.mc.rigidBodies.keys())

            captured_frames = [(name, obj) for name, obj in self.mc.rigidBodies.items() if name in captured_frames]

            for frame_name, obj in captured_frames:
                frame = self.C.frame(frame_name)
                dummy_frame = self.C.frame(self.dummy_frame_name)

                # add constants to get better values
                pos = np.array(obj.position)
                quat = np.array([obj.rotation.w, obj.rotation.x, obj.rotation.y, obj.rotation.z])

                self.C.attach("optitrack_base", self.dummy_frame_name)

                # add offset
                pos = pos + object_offsets[frame_name]

                # set the position and rotation of the object
                dummy_frame.setRelativePosition(pos)
                dummy_frame.setRelativeQuaternion(quat)

                #
                frame.setPosition(dummy_frame.getPosition())
                frame.setQuaternion(dummy_frame.getQuaternion())

                #
                self.info[frame_name]["count"] += 1
                self.info[frame_name]["avg_pos"] += (pos - self.info[frame_name]["avg_pos"]) / self.info[frame_name][
                    "count"]

                # if not step % self.avg_rate:
                #     print(self.info[frame_name]["avg_pos"])
                #     self.info[frame_name]["avg_pos"] = 0
                #     self.info[frame_name]["count"] = 0

                self.last_update = t

    def _check_angular_velocity(self):
        pass


if __name__ == '__main__':

    C, scene_objects = setup_env.setup_tower_env()

    C.view()

    tracker = Tracker(C, ["b1", "b2", "b3"], 1)
    for t in range(100000):
        tracker.update(t)

        time.sleep(0.01)

        # print(C.frame("b1").getPosition())
        print(C.frame("b3").getQuaternion())

    pass
