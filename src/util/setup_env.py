import libry as ry
from os.path import join
import time


def setup_tower_env(num_blocks=1, block_size=(.1, .1, .1)):

    scene = "../rai-robotModels/scenarios/pandasTable.g"

    # setup simulation (Real World)
    R = ry.Config()
    R.addFile(scene)

    # setup configuration (what robot knows)
    C = ry.Config()
    C.addFile(scene)

    positions = (
        (0.1, 0.1),
        (0.3, -0.1)
    )

    block_names = []

    # create blocks
    for o in range(num_blocks):
        name = f"bb{o+1}"
        block = C.addFrame(name)
        block_names.append(name)

        pos = []
        pos.extend(positions[o])
        pos.append(block_size[2]+0.7)

        block.setPosition(pos)
        block.setQuaternion([1, 0, 0, 0])
        block.setShape(ry.ST.box, size=block_size)  # 0.001

    return R, C, block_names


