import numpy as np
import numpy.typing as npt

from data_structures.angles import LegPoseList

class Action():
    target_angles:npt.NDArray[np.float64]        # 1D
    feedforward_torques:npt.NDArray[np.float64]  # 1D

    def __init__(self, target_angles:LegPoseList, feedforward_torques:npt.NDArray[np.float64]):
        self.target_angles       = target_angles.flatten()  # Convert 2D to 1D array
        self.feedforward_torques = feedforward_torques
