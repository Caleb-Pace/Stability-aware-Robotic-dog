import numpy as np
from kinematic_controller.gait_definition import Gait

# # # < Notes > # # #
# 
# Scale: 1.0 = 1 meter
# Legs order: ["FL", "FR", "BL", "BR"]
#
# # # # # # # # # # #

TROT = Gait(
    node_count = 32,
    leg_phase_offset   = np.array([0.0, 0.5, 0.5, 0.0], dtype=float),
    leg_control_points = np.array([
        [[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.08], [0.0, 0.0, 0.1], [0.075, 0.0, 0.08], [0.15, 0.0, 0.0]],
        [[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.08], [0.0, 0.0, 0.1], [0.075, 0.0, 0.08], [0.15, 0.0, 0.0]],
        [[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.08], [0.0, 0.0, 0.1], [0.075, 0.0, 0.08], [0.15, 0.0, 0.0]],
        [[-0.15, 0.0, 0.0], [-0.075, 0.0, 0.08], [0.0, 0.0, 0.1], [0.075, 0.0, 0.08], [0.15, 0.0, 0.0]]
    ], dtype=float),
    body_height        = np.array([0.5], dtype=float)
)
