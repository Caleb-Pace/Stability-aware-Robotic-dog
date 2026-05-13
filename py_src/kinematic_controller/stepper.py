import numpy as np
import numpy.typing as npt
from data_structures import PointList
from kinematic_controller.gaits import LEG_COUNT
from kinematic_controller.gait_definition import Gait


# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(foot_trajectories:npt.NDArray[np.float64], step_num:int) -> PointList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    steps_in_gait = len(foot_trajectories)
    step_num %= steps_in_gait  # Wrap step

    foot_positions = foot_trajectories[step_num]

    return foot_positions

