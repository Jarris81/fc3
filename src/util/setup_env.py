import libry as ry
from os.path import join
import time


def setup_tower_env(num_blocks=1, block_size=(.1, .1, .1, 0.001)):

    path_to_repo ="/home/jason/git/thesis_2020/"
    #scene = "rai/testing/KOMO/switches/model2.g"
    scene = "rai-robotModels/scenarios/pandasTable.g"

    # setup simulation (Real World)
    R = ry.Config()
    R.addFile(path_to_repo+scene)

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(path_to_repo+scene)

    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0],
             [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5], [0, 0.5, 1], [0.5, 1, 0]]

    positions = (
        (0.1, 0.2),
        (0.3, 0.3), #0.8 for infeasible
        (-0.1, -0.2)
    )

    block_names = []

    # create blocks
    for o in range(num_blocks):
        name = f"b{o+1}"

        block = C.addFrame(name)
        # block = C.addFrame(name, "R_gripper")
        block_names.append(name)

        pos_xy = positions[o]
        pos_block = (*pos_xy, block_size[2]/2+0.66)

        # quick hack to see if predicate is feasible
        # if o == 0:
        #     pos_block = 0.3, 0.3, pos_block[2] + block_size[2]

        block.setPosition(pos_block)
        block.setQuaternion([1, 0, 0, 0])
        block.setColor(color[o])
        block.setShape(ry.ST.ssBox, size=block_size)  # 0.001
        block.setContact(1)

        # add a collision box, over block to see if block is free
        place_box = C.addFrame(f"{name}_place_box", parent=name)
        pos_place_box = (*pos_xy, block_size[2]*3/2+0.66)
        place_box.setPosition(pos_place_box)
        place_box.setShape(ry.ST.ssBox, size=block_size)
        place_box.setQuaternion([1, 0, 0, 0])
        place_box.setColor([1, 1, 1, 0.5])  # last value is alpha
        place_box.setContact(0)

    return R, C, block_names


