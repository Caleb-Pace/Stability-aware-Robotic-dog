import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories

# # # < Notes > # # #
# 
# Scale: 1.0 = 1 meter
# Legs order: ["FR", "FL", "RR", "RL"]
#
# # # # # # # # # # #

# TODO: Handle different point counts per leg
trot_ctrl_pts = np.array([
    np.array([                                                        [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([                                                        [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08]]),
])
trot_ctrl_pts_shallow = np.array([
    np.array([                                                        [0.15, 0.0, 0.03], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, 0.0, 0.03], [0.1, 0.0, 0.05], [0.15, 0.0, 0.03]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.03], [0.1, 0.0, 0.05], [0.15, 0.0, 0.03], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.03], [0.1, 0.0, 0.05], [0.15, 0.0, 0.03], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([                                                        [0.15, 0.0, 0.03], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, 0.0, 0.03], [0.1, 0.0, 0.05], [0.15, 0.0, 0.03]]),
])

TROT = Gait(
    transition_in   = LegTrajectories(
        sample_count       = 64,
        heights            = np.array([0.3], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
        leg_control_points = np.array([
            np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1]]),
            np.array([[0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1]]),
        ], dtype=object)
    ),
    transistion_out = LegTrajectories(
        sample_count       = 64, 
        heights            = np.array([0.3], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
        leg_control_points = np.array([
            np.array([[0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0]]),
            np.array([[0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0]]),
            np.array([[0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0]]),
        ], dtype=object)
    ),
    loop            = LegTrajectories(
        sample_count       = 192,
        heights            = np.array([0.3], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)
