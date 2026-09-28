import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories


SIT = LegTrajectories(
    sample_count       = 8,
    heights            = np.array([0.3, 0.05], dtype=float),
    orientations       = np.array([[0, 0, 0]], dtype=float),
    leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
    leg_control_points = np.array([
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0]]),
    ], dtype=object)
)