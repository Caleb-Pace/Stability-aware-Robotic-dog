import numpy as np
import numpy.typing as npt   # TODO: Remove, unused
from data_structures import PointList
from kinematic_controller.gait_definition import Gait, LEG_COUNT


# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(gait:Gait, step_num:int) -> PointList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    # TODO: Uncomment, disabled for testing
    # steps_in_gait = len(foot_trajectories[0])
    # step_num %= steps_in_gait  # Wrap step

    # Need time anchors to determine phase offset

    for i in range(LEG_COUNT):
        foot_positions[i] = gait.foot_trajectories[i][step_num]

    return foot_positions

# TODO: Add method to offset relative foot positions (for displaying)
