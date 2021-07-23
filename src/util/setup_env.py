import libry as ry
import numpy as np
import util.constants as constants
from os.path import join
import time

path_to_repo = "/home/jason/git/thesis_2020/"

table_height = 0.65


def _setup(dual=False, num_blocks=1, positions=((-0.6, -0.2)),  block_size=(.06, .06, .06, 0.001)):
    # scene = "rai/testing/KOMO/switches/model2.g"
    if dual:
        scene = "rai-robotModels/scenarios/pandasTable.g"
    else:
        scene = "rai-robotModels/scenarios/pandaSingle.g"

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_repo + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    scene_objects = {constants.type_gripper: ["R_gripper"], constants.type_block: []}

    # create blocks
    for o in range(num_blocks):
        name = f"b{o + 1}"

        block = C.addFrame(name)
        scene_objects[constants.type_block].append(name)

        pos_xy = positions[o]
        pos_block = (*pos_xy, block_size[2] / 2 + 0.635)

        # quick hack to see if predicate is feasible
        # if o == 0:
        #     pos_block = 0.3, 0.3, pos_block[2] + block_size[2]

        block.setPosition(pos_block)
        block.setQuaternion([1, 0, 0, 0])
        block.setColor(color[o])
        block.setMass(1)
        block.setShape(ry.ST.box, size=block_size[:-1])  # 0.001
        block.setContact(1)

    # S = C.simulation(ry.SimulatorEngine.bullet, True)
    return C, scene_objects




def setup_tower_env():
    positions = (
        (-0.6, -0.2),
        (-0.4, -0.1),  # 0.8 for infeasible
        (-0.5, -0.5),
    )

    return _setup(num_blocks=3, positions=positions)


def setup_pick_and_place_env():
    C, scene_objects = _setup(dual=True, num_blocks=1)

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


def setup_hand_over_env(block_size=(.06, .06, .06, 0.001)):
    scene = "rai-robotModels/scenarios/pandasTable.g"

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_repo + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    positions = (
        (0.6, 0.1),
        (0.4, 0.0),  # 0.8 for infeasible
        (0.2, 0.2),
    )

    scene_objects = {constants.type_gripper: ["R_gripper"], constants.type_block: []}

    # create blocks
    name = "b1"

    block = C.addFrame(name)
    scene_objects[constants.type_block].append(name)

    pos_xy = positions[0]
    pos_block = (*pos_xy, block_size[2] / 2 + 0.635)

    # quick hack to see if predicate is feasible
    # if o == 0:
    #     pos_block = 0.3, 0.3, pos_block[2] + block_size[2]

    block.setPosition(pos_block)
    block.setQuaternion([1, 0, 0, 0])
    block.setColor(color[0])
    block.setMass(1)
    block.setShape(ry.ST.box, size=block_size[:-1])  # 0.001
    block.setContact(1)

    return C, scene_objects


def setup_bottle_open_env():

    scene = "rai-robotModels/scenarios/pandasTable.g"

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_repo + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    bottle_height, bottle_radius = 0.09, 0.03
    cap_height, cap_radius = 0.02, 0.03

    pos = 0.0, 0.2

    position_bottle = (*pos, table_height + bottle_height / 2)
    position_cap = (*pos, table_height + bottle_height + cap_height / 2)

    bottle = C.addFrame("bottle")
    bottle.setPosition(position_bottle)
    bottle.setQuaternion([1, 0, 0, 0])
    bottle.setColor((0, 0, 1))
    bottle.setMass(1)
    bottle.setShape(ry.ST.cylinder, size=(bottle_height, bottle_radius))  # 0.001
    bottle.setContact(1)
    
    cap = C.addFrame("cap")
    cap.setPosition(position_cap)
    cap.setQuaternion([1, 0, 0, 0])
    cap.setColor((1, 0, 0))
    cap.setMass(1)
    cap.setShape(ry.ST.cylinder, size=(cap_height, cap_radius))  # 0.001
    cap.setContact(1)

    C.attach("bottle", "cap")

    scene_objects = {constants.type_gripper: ["R_gripper"], constants.type_bottle: ["bottle"]}

    return C, scene_objects

