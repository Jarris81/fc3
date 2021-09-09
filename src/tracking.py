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

constant_offset = np.array([0, -0.03, constants.table_height])


class Tracker:

    def __init__(self, C, frame_names, rate):

        self.C = C
        self.frame_names = set(frame_names)
        self.rate = rate

        self.mc = motioncapture.MotionCaptureOptitrack("130.149.82.29")

        self.info = {frame: {"avg_pos": np.zeros(3), "count": 0} for frame in self.frame_names}

        self.avg_rate = 100

    def update(self, step):

        if not step % self.rate:

            self.mc.waitForNextFrame()

            # get intersection of frames
            captured_frames = self.frame_names.intersection(self.mc.rigidBodies.keys())

            captured_frames = [(name, obj) for name, obj in self.mc.rigidBodies.items() if name in captured_frames]

            for frame_name, obj in captured_frames:
                frame = self.C.frame(frame_name)

                # add constants to get better values
                pos = np.array(obj.position)
                quat = np.array([obj.rotation.w, obj.rotation.x, obj.rotation.y, obj.rotation.z])

                # add offset
                pos = pos + object_offsets[frame_name] + constant_offset

                # set the position and rotation of the object
                frame.setPosition(pos)
                frame.setQuaternion(quat)

                #
                self.info[frame_name]["count"] += 1
                self.info[frame_name]["avg_pos"] += (pos-self.info[frame_name]["avg_pos"])/self.info[frame_name]["count"]

                if not step % self.avg_rate:
                    print(self.info[frame_name]["avg_pos"])
                    self.info[frame_name]["avg_pos"] = 0
                    self.info[frame_name]["count"] = 0



    def _check_angular_velocity(self):
        pass


if __name__ == '__main__':

    C, scene_objects = setup_env.setup_tower_env()

    test = C.frame("test")
    C.view()

    tracker = Tracker(C, ["b1"], 1)
    for t in range(100000):
        tracker.update(t)

        time.sleep(0.01)

    pass
