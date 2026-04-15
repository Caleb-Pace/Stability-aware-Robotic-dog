import numpy as np
import numpy.typing as npt
from kinematic_controller.gait_definition import Gait
from kinematic_controller.gaits import LEG_COUNT

# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(gait:Gait, step_num:int) -> npt.NDArray[np.float64]:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    steps_in_gait = 2  # TODO: Calculate and attach to gait (can be found by duty factor)

    # TODO: Fix, not using duty factor
    for i in range(LEG_COUNT):
        if (gait.leg_phases_offset[i] * steps_in_gait) == step_num:
            foot_positions[i,0] = step_num+1

    # gait.leg_phases_offset
    # gait.leg_duty_factor

    return foot_positions

