import numpy as np
import numpy.typing as npt
from itertools import chain

from data_structures.angles import LegPoseList

class Action():
    target_angles:npt.NDArray[np.float64]        # 1D
    feedforward_torques:npt.NDArray[np.float64]  # 1D

    def __init__(self, target_angles:LegPoseList, feedforward_torques:npt.NDArray[np.float64]):
        self.target_angles       = np.fromiter(chain.from_iterable(target_angles), dtype=np.float64)  # Convert 2D to 1D array
        self.feedforward_torques = feedforward_torques
