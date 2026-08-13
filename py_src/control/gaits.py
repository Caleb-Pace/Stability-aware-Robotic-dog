import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories

# # # < Notes > # # #
# 
# Scale: 1.0 = 1 meter
# Legs order: ["FR", "FL", "RR", "RL"]
#
# # # # # # # # # # #

TROT = Gait(
    body_height        = np.array([0.5], dtype=float),
    transition_in   = LegTrajectories(
        sample_count       = 64, 
        leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
        leg_control_points = np.array([
            np.array([[0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2]]),
            np.array([[0.0, 0.0, -0.3]]),
            np.array([[0.0, 0.0, -0.3]]),
            np.array([[0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2]]),
        ], dtype=object)
    ),
    transistion_out = LegTrajectories(
        sample_count       = 64, 
        leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
        leg_control_points = np.array([
            np.array([[0.0, 0.0, -0.3]]),
            np.array([[0.0, 0.0, -0.3]]),
            np.array([[0.1, 0.0, -0.2], [0.15, 0.0, -0.22], [0.2, 0.0, -0.3]]),
            np.array([[0.1, 0.0, -0.2], [0.15, 0.0, -0.22], [0.2, 0.0, -0.3]]),
        ], dtype=object)
    ),
    loop            = LegTrajectories(
        sample_count       = 16,
        leg_phase_offset   = np.array([0.0, 0.0, 0.0, 0.0], dtype=float),
        leg_control_points = np.array([
            np.array([                                                        [0.15, 0.0, -0.22], [0.2, 0.0, -0.3], [0.15, 0.0, -0.3], [0.1, 0.0, -0.3], [0.05, 0.0, -0.3], [0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2]]),
            np.array([[0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2], [0.15, 0.0, -0.22], [0.2, 0.0, -0.3], [0.15, 0.0, -0.3], [0.1, 0.0, -0.3], [0.05, 0.0, -0.3], [0.0, 0.0, -0.3]]),
            np.array([[0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2], [0.15, 0.0, -0.22], [0.2, 0.0, -0.3], [0.15, 0.0, -0.3], [0.1, 0.0, -0.3], [0.05, 0.0, -0.3], [0.0, 0.0, -0.3]]),
            np.array([                                                        [0.15, 0.0, -0.22], [0.2, 0.0, -0.3], [0.15, 0.0, -0.3], [0.1, 0.0, -0.3], [0.05, 0.0, -0.3], [0.0, 0.0, -0.3], [0.05, 0.0, -0.22], [0.1, 0.0, -0.2]]),
        ], dtype=object)
    )
)
