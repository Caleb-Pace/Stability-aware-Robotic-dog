import numpy as np
import numpy.typing as npt
from kinematic_controller.gait_definition import Gait
from kinematic_controller.gaits import LEG_COUNT

def step(gait:Gait, num:int) -> npt.NDArray[np.float64]:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    # gait.leg_phases_offset
    # gait.leg_duty_factor

    return foot_positions

