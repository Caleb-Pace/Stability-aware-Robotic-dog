import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories
from data_structures.constants import STAND_HEIGHT

# # # < Notes > # # #
# 
# Scale:       1.0 = 1 meter
# Legs order:  ["FR", "FL", "RR", "RL"]
# Orientation: [roll, pitch, yaw] (in radians)
#
# # # # # # # # # # #


### Tuning parameters
LEAN_FORWARD_X = -0.06
TRANSITION_STEP_COUNT = 12
LOOP_STEP_COUNT = 64+32
STEP_LENGTH_SCALE = 1
STEP_HEIGHT_SCALE = 1.5
# STEP_LENGTH_SCALE = 1.5
# STEP_HEIGHT_SCALE = 1.5
# STEP_LENGTH_SCALE = 0.50
# STEP_HEIGHT_SCALE = 0.60
# PITCH_COMP = -0.03
# PITCH_COMP = 0.02
# PITCH_COMP = 0.03
PITCH_COMP = 0.0
# PITCH_COMP = 0.04
# PITCH_COMP = 0.05
# PITCH_COMP = 0.06
# BREADTH = -0.025
BREADTH = -0.01
# BREADTH =  0.015
# BREADTH =  0.15

### Control Points
# trot_ctrl_pts = np.array([
#     np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
# ] * 4)  # Original
# trot_ctrl_pts = np.array([
#     np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
# ] * 4)  # Tip toe
# trot_ctrl_pts = np.array([
#     np.array([[0.0, BREADTH, 0.0], [0.04, BREADTH, 0.06], [0.09, BREADTH, 0.08], [0.14, BREADTH, 0.06], [0.18, BREADTH, 0.01], [0.14, BREADTH, 0.0], [0.09, BREADTH, 0.0], [0.04, BREADTH, 0.0], [0.0, BREADTH, 0.0]]),
# ] * 4)  # Maanas gait copy
trot_ctrl_pts = np.array([
    np.array([[-0.06, BREADTH, 0.0], [-0.02, BREADTH, 0.06], [0.03, BREADTH, 0.08], [0.07, BREADTH, 0.06], [0.10, BREADTH, 0.01], [0.05, BREADTH, 0.0], [0.00, BREADTH, 0.0], [-0.05, BREADTH, 0.0], [-0.06, BREADTH, 0.0]]),
] * 4)  # Dynamics one

### Tuning
for i, _leg_points in enumerate(trot_ctrl_pts):
    # Y
    if i % 2 == 0:
        _leg_points[:, 1] *= -1

    # X
    _leg_points[:, 0] *= STEP_LENGTH_SCALE
    
    # Z
    _leg_points[:, 2] *= STEP_HEIGHT_SCALE

TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([STAND_HEIGHT], dtype=float),
        orientations       = np.array([[0, PITCH_COMP, 0]], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)
