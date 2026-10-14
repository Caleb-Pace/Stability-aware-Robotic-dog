import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories

# # # < Notes > # # #
# 
# Scale: 1.0 = 1 meter
# Legs order: ["FR", "FL", "RR", "RL"]
#
# # # # # # # # # # #


# # Scale steps
TRANSITION_STEP_COUNT = 12
LOOP_STEP_COUNT = 64+32
STEP_LENGTH_SCALE = 1
STEP_HEIGHT_SCALE = 1
STAND_HEIGHT = 0.32
# BREADTH = -0.025
# BREADTH = -0.01
BREADTH =  0.015
# BREADTH =  0.15

# trot_ctrl_pts = np.array([
#     np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
# ] * 4)
# trot_ctrl_pts = np.array([
#     np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
# ] * 4)
trot_ctrl_pts = np.array([
    np.array([[0.0, BREADTH, 0.0], [0.04, BREADTH, 0.06], [0.09, BREADTH, 0.08], [0.14, BREADTH, 0.06], [0.18, BREADTH, 0.01], [0.14, BREADTH, 0.0], [0.09, BREADTH, 0.0], [0.04, BREADTH, 0.0], [0.0, BREADTH, 0.0]]),
] * 4)

# Keep the trot conservative while giving swing legs enough clearance.
for i, _leg_points in enumerate(trot_ctrl_pts):
    if i % 2 == 0:
        _leg_points[:, 1] *= -1
    
    _leg_points[:, 0] *= STEP_LENGTH_SCALE
    _leg_points[:, 2] *= STEP_HEIGHT_SCALE

TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([STAND_HEIGHT], dtype=float),
        orientations       = np.array([[0, 0.03, 0]], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)
