import numpy as np
import numpy.typing as npt

from data_structures.constants import LEG_COUNT, JOINT_COUNT  # Related constants
from data_structures.leg_trajectories import LegTrajectories


class Gait:
    transition_in:LegTrajectories
    transistion_out:LegTrajectories
    loop:LegTrajectories

    body_height: npt.NDArray[np.float64]  # (Heights)
    # body_pitch: float  # Future
    # body_roll: float  # Future


    def __init__(self, body_height:npt.NDArray[np.float64],
                 transition_in:LegTrajectories,
                 transistion_out:LegTrajectories,
                 loop:LegTrajectories):
        # Shape check: (Heights)
        if body_height.ndim != 1:
            raise ValueError(f"Invalid shape {body_height.shape}. Must be (*,)!")
        self.body_height = body_height

        self.transition_in   = transition_in
        self.transistion_out = transistion_out
        self.loop            = loop
