import numpy as np
from kinematic_controller.gait_definition import Gait

TROT = Gait(
    node_count = 32,
    leg_phase_offset   = np.array([0.0, 0.5, 0.5, 0.0], dtype=float),
    leg_control_points = np.array([
        [[-5.0, 0.0, 0.0], [-2.5, 0.0, 5.0], [0.0, 0.0, 7.5], [2.5, 0.0, 5.0], [5.0, 0.0, 0.0]],
        [[-5.0, 0.0, 0.0], [-2.5, 0.0, 5.0], [0.0, 0.0, 7.5], [2.5, 0.0, 5.0], [5.0, 0.0, 0.0]],
        [[-5.0, 0.0, 0.0], [-2.5, 0.0, 5.0], [0.0, 0.0, 7.5], [2.5, 0.0, 5.0], [5.0, 0.0, 0.0]],
        [[-5.0, 0.0, 0.0], [-2.5, 0.0, 5.0], [0.0, 0.0, 7.5], [2.5, 0.0, 5.0], [5.0, 0.0, 0.0]]
    ], dtype=float),
    body_height        = np.array([0.5], dtype=float)
)
