import libry as ry
import numpy as np
import util.constants as constants
from os.path import join
import time

table_height = 0.65


def setup_tower_env(num_blocks=3, block_size=(.06, .06, .06, 0.001)):
    path_to_repo = "/home/jason/git/thesis_2020/"
    # scene = "rai/testing/KOMO/switches/model2.g"
    # scene = "rai-robotModels/scenarios/pandasTable.g"
    scene = "rai-robotModels/scenarios/pandaSingle.g"

    # setup simulation (Real World)

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_repo + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    positions = (
        (0.6, -0.2),
        (0.4, 0.0),  # 0.8 for infeasible
        (0.1, 0.3),
    )

    scene_objects = {constants.type_gripper: ["R_gripper"], constants.type_block: []}

    # create blocks
    for o in range(num_blocks):
        name = f"b{o + 1}"

        block = C.addFrame(name)
        scene_objects[constants.type_block].append(name)

        pos_xy = positions[o]
        pos_block = (*pos_xy, block_size[2] / 2 + 0.65)

        # quick hack to see if predicate is feasible
        # if o == 0:
        #     pos_block = 0.3, 0.3, pos_block[2] + block_size[2]

        block.setPosition(pos_block)
        block.setQuaternion([1, 0, 0, 0])
        block.setColor(color[o])
        block.setMass(1)
        block.setShape(ry.ST.box, size=block_size[:-1])  # 0.001
        block.setContact(1)

    #S = C.simulation(ry.SimulatorEngine.bullet, True)
    return C, scene_objects


def setup_pick_and_place_env(block_size=(.1, .1, .1, 0.001)):
    C, scene_objects = setup_tower_env(1)

    # radius = 0.02
    # stick_length = 0.5
    # stick_pos = np.array([0.5, 0.5, table_height + radius])
    # scene_objects[constants.type_stick] = ["stick"]
    # stick = C.addFrame("stick")
    # stick.setQuaternion([1, 1, 0, 0])
    # stick.setPosition(stick_pos)
    # stick.setColor([1, 1, 1])
    # stick.setMass(1)
    # stick.setShape(ry.ST.cylinder, size=[stick_length, radius])  # 0.001
    # stick.setContact(1)
    #
    # stick_handle_length = 0.2
    # stick_handle = C.addFrame("stickHandle", parent="stick")
    # stick_handle.setQuaternion([1, 0, 1, 0])
    # stick_handle_pos = stick_pos + np.array([-stick_handle_length / 2, stick_length / 2, 0])
    # stick_handle.setPosition(stick_handle_pos)
    # stick_handle.setColor([1, 1, 1])
    # stick_handle.setMass(1)
    # stick_handle.setShape(ry.ST.cylinder, size=[stick_handle_length, radius])  # 0.001
    # stick_handle.setContact(1)

    #S = C.simulation(ry.SimulatorEngine.bullet, True)

    return C, scene_objects
