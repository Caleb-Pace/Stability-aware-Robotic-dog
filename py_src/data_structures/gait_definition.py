import numpy as np
import numpy.typing as npt

from data_structures.constants import LEG_COUNT, JOINT_COUNT  # Related constants
from data_structures.leg_trajectories import LegTrajectories


class Gait:
    transition_in:LegTrajectories
    transistion_out:LegTrajectories
    loop:LegTrajectories


    def __init__(self,
                 transition_in:LegTrajectories,
                 transistion_out:LegTrajectories,
                 loop:LegTrajectories):

        self.transition_in   = transition_in
        self.transistion_out = transistion_out
        self.loop            = loop
