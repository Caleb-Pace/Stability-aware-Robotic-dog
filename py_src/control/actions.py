import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories


SIT = LegTrajectories(
    sample_count       = 2,
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, -0.3]]),
        np.array([[0.0, 0.0, -0.3]]),
        np.array([[0.0, 0.0, -0.3]]),
        np.array([[0.0, 0.0, -0.3]]),
    ], dtype=object)
)