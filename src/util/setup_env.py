import libry as ry
import numpy as np
import util.constants as constants
from os.path import join
import time

path_to_botop = "../../../botop/"


def _setup(dual=False, num_blocks=1, positions=((-0.6, -0.2))):
    # scene = "rai/testing/KOMO/switches/model2.g"
    scene = "rai-robotModels/scenarios/pandasTable.g"
    scene_objects = {constants.type_gripper: ["r_gripper"], constants.type_block: []}

    if dual:
        scene_objects[constants.type_gripper].append("l_gripper")

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_botop + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    block_sizes = [
        (.06, .06, .06,),
        (.07, .08, .06),
        (.09, .14, .05),
    ]



    # create blocks
    for o in range(num_blocks):
        name = f"b{o + 1}"

        block = C.addFrame(name)
        scene_objects[constants.type_block].append(name)

        pos_xy = positions[o]
        pos_block = (*pos_xy, block_sizes[o][2] / 2 + constants.table_height)

        # quick hack to see if predicate is feasible
        # if o == 0:
        #     pos_block = 0.3, 0.3, pos_block[2] + block_size[2]

        block.setPosition(pos_block)
        block.setQuaternion([1, 0, 0, 0])
        block.setColor(color[o])
        block.setMass(1)
        block.setShape(ry.ST.box, size=block_sizes[o])  # 0.001
        block.setContact(1)

    # S = C.simulation(ry.SimulatorEngine.bullet, True)
    return C, scene_objects


def setup_tower_env():
    positions = (
        (0.7, 0.4),
        (0.4, 0.3),
        (0.6, 0.2)

    )

    return _setup(num_blocks=3, positions=positions)


def setup_hand_over_env():
    positions = (
        (0.4, 0.0),
    )
    C, scene_objects = _setup(dual=True, num_blocks=1, positions=positions)

    return C, scene_objects


def setup_stick_pull_env():
    positions = (
        (0.8, 0.3),
    )

    C, scene_objects = _setup(dual=False, num_blocks=1, positions=positions)

    stick_length = 0.5
    stick_th = 0.03

    stick_x, stick_y = 0.9, -0.1

    stick = C.addFrame("stick")
    stick.setPosition((stick_x, stick_y, constants.table_height+stick_th/2))
    stick.setQuaternion([1, 0, 0, 0])
    stick.setColor((0, 0, 1))
    stick.setMass(1)
    stick.setShape(ry.ST.box, size=(stick_th, stick_length, stick_th))  # 0.001
    stick.setContact(1)

    handle_length = 0.2

    stick_handle = C.addFrame("stickHandle", parent="stick")
    stick_handle.setPosition((stick_x - (handle_length-stick_th)/2,
                              stick_y + (stick_length-stick_th)/2,
                              constants.table_height+stick_th/2))
    stick_handle.setQuaternion([1, 0, 0, 0])
    stick_handle.setColor((0, 0, 1))
    stick_handle.setMass(1)
    stick_handle.setShape(ry.ST.box, size=(handle_length, stick_th, stick_th))  # 0.001
    stick_handle.setContact(1)

    scene_objects[constants.type_stick] = ("stick", )

    return C, scene_objects


def setup_bottle_open_env():

    scene = "rai-robotModels/scenarios/pandasTable.g"

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_botop + scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    bottle_height, bottle_radius = 0.09, 0.03
    cap_height, cap_radius = 0.02, 0.03

    pos = 0.0, 0.2

    position_bottle = (*pos, constants.table_height + bottle_height / 2)
    position_cap = (*pos, constants.table_height + bottle_height + cap_height / 2)

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

    scene_objects = {constants.type_gripper: ["l_gripper"], constants.type_bottle: ["bottle"]}

    return C, scene_objects

