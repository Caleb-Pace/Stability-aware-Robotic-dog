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
# COMP_PITCH = np.pi/12 # 15% 
COMP_PITCH = np.pi/18 # 10% 
# COMP_PITCH = 0 # 10% 

# # Scale steps
# STEP_SCALE = 2
# STEP_SCALE = 0.5
TRANSITION_STEP_COUNT = 4
LOOP_STEP_COUNT = 256
STEP_LENGTH_SCALE = 0.5
STEP_HEIGHT_SCALE = 0.6
STAND_HEIGHT = 0.32


trot_ctrl_pts = np.array([
    np.array([[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.10], [0.0, 0.0, 0.14], [0.075, 0.0, 0.10], [0.15, 0.0, 0.0], [0.075, 0.0, 0.0], [0.0, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.15, 0.0, 0.0]]),
    np.array([[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.10], [0.0, 0.0, 0.14], [0.075, 0.0, 0.10], [0.15, 0.0, 0.0], [0.075, 0.0, 0.0], [0.0, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.15, 0.0, 0.0]]),
    np.array([[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.10], [0.0, 0.0, 0.14], [0.075, 0.0, 0.10], [0.15, 0.0, 0.0], [0.075, 0.0, 0.0], [0.0, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.15, 0.0, 0.0]]),
    np.array([[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.10], [0.0, 0.0, 0.14], [0.075, 0.0, 0.10], [0.15, 0.0, 0.0], [0.075, 0.0, 0.0], [0.0, 0.0, 0.0], [-0.075, 0.0, 0.0], [-0.15, 0.0, 0.0]]),
])

# Use a full front-to-rear stance sweep so each planted foot can push the body.
for _leg_points in trot_ctrl_pts:
    _leg_points[:, 0] *= STEP_LENGTH_SCALE
    _leg_points[:, 2] *= STEP_HEIGHT_SCALE

TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([STAND_HEIGHT], dtype=float),
        orientations       = np.array([[0, 0, 0]], dtype=float),
        # orientations       = np.array([[0, COMP_PITCH, 0]], dtype=float),
        # leg_phase_offset   = np.array([0, np.pi/2, np.pi/2, 0], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)
