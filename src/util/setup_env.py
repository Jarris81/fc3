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
        (0.3, 0.3),
        (-0.1, -0.2)
    )

    block_names = []

    # create blocks
    for o in range(num_blocks):
        name = f"b{o+1}"
        block = C.addFrame(name)
        block_names.append(name)

        pos = []
        pos.extend(positions[o])
        pos.append(block_size[2]/2+0.65)

        block.setPosition(pos)
        block.setQuaternion([1, 0, 0, 0])
        block.setColor(color[o])
        block.setShape(ry.ST.ssBox, size=block_size)  # 0.001
        block.setContact(1)

    return R, C, block_names


