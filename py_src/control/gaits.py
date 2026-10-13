import numpy as np
from data_structures.gait_definition import Gait
from data_structures.leg_trajectories import LegTrajectories

# # # < Notes > # # #
# 
# Scale: 1.0 = 1 meter
# Legs order: ["FR", "FL", "RR", "RL"]
#
# # # # # # # # # # #

# Pitch to compensate for weight distribution
# COMP_PITCH = np.pi/18 # 10% 
# COMP_PITCH = np.pi/12 # 15% 
# COMP_PITCH = np.pi/36 # 5% 
# COMP_PITCH = 0 # 0% 

# # Scale steps
# STEP_SCALE = 2
# STEP_SCALE = 0.5
# LOOP_STEP_COUNT = 12


trot_ctrl_pts = np.array([
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.08], [0.1, 0.0, 0.1], [0.15, 0.0, 0.08], [0.2, 0.0, 0.0], [0.15, 0.0, 0.0], [0.1, 0.0, 0.0], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]]),
])

LOOP_STEP_COUNT = 192
TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([0.3], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        # orientations       = np.array([[0, COMP_PITCH, 0]], dtype=float),
        # leg_phase_offset   = np.array([0, np.pi/2, np.pi/2, 0], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)

LOOP_STEP_COUNT = 256
MAANAS_TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([0.32], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = np.array([
            np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.06], [0.09, 0.0, 0.08], [0.14, 0.0, 0.06], [0.18, 0.0, 0.0], [0.14, 0.0, 0.0], [0.09, 0.0, 0.0], [0.04, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        ])
    )
)