import time

from tracking import Tracker

import util.setup_env as setup_env

C, scene_objects = setup_env.setup_tower_env()

C.view()

tracker = Tracker(C, ["b1"], 10)
for t in range(100000):

    tracker.update(t)

    time.sleep(0.01)

