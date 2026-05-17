import math
import numpy as np
import numpy.typing as npt   # TODO: Remove, unused
from data_structures import PointList
from kinematic_controller.gait_definition import Gait, LEG_COUNT


# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(gait:Gait, step_num:int) -> PointList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    # TODO: Uncomment, disabled for testing
    # steps_in_gait = len(foot_trajectories[0])
    # step_num %= steps_in_gait  # Wrap step

    # Need time anchors to determine phase offset

    for i in range(LEG_COUNT):
        phase_offset_as_steps = math.ceil(gait.leg_phase_offset[i] * gait.steps_in_gait)
        movement_length_in_steps = len(gait.time_anchors[i])

        print(f"Leg{i} | step{step_num} | gaitFT[{(step_num - phase_offset_as_steps)}] | Mvmnt: ( pO{{{phase_offset_as_steps}}} + mvL{{{movement_length_in_steps}}} ) = {(phase_offset_as_steps + movement_length_in_steps)} / {gait.steps_in_gait} | tRef: {gait.time_reference}")  # TODO: Remove, for debugging

        if step_num < phase_offset_as_steps:  # Before
            foot_positions[i] = gait.foot_trajectories[i][0]  # Starting position
        elif step_num >= (phase_offset_as_steps + movement_length_in_steps):  # After
            foot_positions[i] = gait.foot_trajectories[i][-1]  # End position
        else:
            foot_positions[i] = gait.foot_trajectories[i][(step_num - phase_offset_as_steps)]
    
    return foot_positions

# TODO: Add method to offset relative foot positions (for displaying)
