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
LOOP_STEP_COUNT = 120
STEP_LENGTH_SCALE = 0.7
STEP_HEIGHT_SCALE = 0.75
STAND_HEIGHT = 0.32


step_profile = np.array([
    [-0.15, 0.0, 0.0], [-0.075, 0.0, 0.10], [0.0, 0.0, 0.14],
    [0.075, 0.0, 0.10], [0.15, 0.0, 0.0],
    [0.125, 0.0, 0.0], [0.10, 0.0, 0.0], [0.075, 0.0, 0.0],
    [0.05, 0.0, 0.0], [0.025, 0.0, 0.0], [0.0, 0.0, 0.0],
    [-0.025, 0.0, 0.0], [-0.05, 0.0, 0.0], [-0.075, 0.0, 0.0],
    [-0.10, 0.0, 0.0], [-0.125, 0.0, 0.0], [-0.15, 0.0, 0.0],
])
trot_ctrl_pts = np.array(
    [step_profile.copy() for _ in range(4)],
    dtype=object,
)
for _leg, _side in enumerate((1.0, -1.0, 1.0, -1.0)):
    trot_ctrl_pts[_leg][:, 1] = 0.018 * _side

# Use a full front-to-rear stance sweep so each planted foot can push the body.
for _leg_points in trot_ctrl_pts:
    _leg_points[:, 0] *= STEP_LENGTH_SCALE
    _leg_points[:, 2] *= STEP_HEIGHT_SCALE

TROT = Gait(
    loop            = LegTrajectories(
        sample_count       = LOOP_STEP_COUNT,
        heights            = np.array([STAND_HEIGHT], dtype=float),
        orientations       = np.array([[0, 0.08, 0]], dtype=float),
        # orientations       = np.array([[0, COMP_PITCH, 0]], dtype=float),
        # leg_phase_offset   = np.array([0, np.pi/2, np.pi/2, 0], dtype=float),
        leg_phase_offset   = np.array([0, np.pi, np.pi, 0], dtype=float),
        leg_control_points = trot_ctrl_pts
    )
)
