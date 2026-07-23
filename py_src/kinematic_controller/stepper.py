import math
import numpy as np
from data_structures import Point3DList
from kinematic_controller.gait_definition import Gait, LEG_COUNT
from kinematic_controller.ik_solver import _LEG_OFFSETS_FROM_BODY_ORIGIN


# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(gait:Gait, step_num:int) -> Point3DList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    # TODO: Uncomment, disabled for testing
    # steps_in_gait = len(foot_trajectories[0])
    # step_num %= steps_in_gait  # Wrap step

    # Need time anchors to determine phase offset

    for i in range(LEG_COUNT):
        movement_length_in_steps = len(gait.time_anchors[i])
        phase_offset_as_steps = math.ceil(gait.steps_in_gait * gait.leg_phase_offset[i])

        # print(f"Leg{i} | step{step_num} | gaitFT[{(step_num - phase_offset_as_steps)}] | Mvmnt: ( pO{{{phase_offset_as_steps}}} + mvL{{{movement_length_in_steps}}} ) = {(phase_offset_as_steps + movement_length_in_steps)} / {gait.steps_in_gait} | tRef: {gait.time_reference}")  # TODO: Remove, for debugging

        if step_num < phase_offset_as_steps:  # Before
            foot_positions[i] = gait.foot_trajectories[i][0]  # Starting position
        elif step_num >= (phase_offset_as_steps + movement_length_in_steps):  # After
            foot_positions[i] = gait.foot_trajectories[i][-1]  # End position
        else:
            foot_positions[i] = gait.foot_trajectories[i][(step_num - phase_offset_as_steps)]
    
    return foot_positions

def apply_offset(foot_positions:Point3DList) -> Point3DList:
    if len(foot_positions) != LEG_COUNT:
        raise ValueError(f"{{foot_positions}} must be equal to the {{LEG_COUNT}}! ({foot_positions} == {LEG_COUNT})")
    
    return (foot_positions + _LEG_OFFSETS_FROM_BODY_ORIGIN)  # Translate foot positions
