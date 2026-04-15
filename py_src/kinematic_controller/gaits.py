import numpy as np
from kinematic_controller.gait_definition import Gait

LEG_COUNT:int = 4

TROT = Gait(
    leg_phases_offset = np.array([[0.0], [0.5], [0.5], [0.0]], dtype=float),
    leg_duty_factor   = np.array([[0.5], [0.5], [0.5], [0.5]], dtype=float),
    duty_cycle_length = 0.8,
    body_height = 0.5
)
