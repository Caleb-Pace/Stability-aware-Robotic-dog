import math
import numpy as np

from data_structures import Point3DList
from data_structures.leg_trajectories import LegTrajectories
from data_structures.gait_definition import LEG_COUNT

from kinematics.ik_solver import LEG_OFFSETS_FROM_BODY_ORIGIN


# TODO: Needs to take in current position or preserve position
# TODO: Implement safety checks to ensure gait arrays are correct length
def step(trajectory:LegTrajectories, step_num:int) -> Point3DList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)
    motion_path = trajectory

    # TODO: Uncomment, disabled for testing
    # steps_in_gait = len(foot_trajectories[0])
    # step_num %= steps_in_gait  # Wrap step

    # Need time anchors to determine phase offset

    phase_offsets_as_steps = np.ceil((motion_path.steps_in_gait * motion_path._phase_offset))
    
    for leg in range(LEG_COUNT):
        movement_length_in_steps = len(motion_path.time_anchors[leg])

        # print(f"Leg{i} | step{step_num} | gaitFT[{(step_num - phase_offset_as_steps)}] | Mvmnt: ( pO{{{phase_offset_as_steps}}} + mvL{{{movement_length_in_steps}}} ) = {(phase_offset_as_steps + movement_length_in_steps)} / {gait.steps_in_gait} | tRef: {gait.time_reference}")  # TODO: Remove, for debugging

        if step_num < phase_offsets_as_steps[leg]:  # Before
            foot_positions[leg] = motion_path.foot_trajectories[leg][0]  # Starting position
        elif step_num >= (phase_offsets_as_steps[leg] + movement_length_in_steps):  # After
            foot_positions[leg] = motion_path.foot_trajectories[leg][-1]  # End position
        else:
            foot_positions[leg] = motion_path.foot_trajectories[leg][int(step_num - phase_offsets_as_steps[leg])]
    
    return foot_positions

def apply_offset(foot_positions:Point3DList) -> Point3DList:
    if len(foot_positions) != LEG_COUNT:
        raise ValueError(f"{{foot_positions}} must be equal to the {{LEG_COUNT}}! ({foot_positions} == {LEG_COUNT})")
    
    return (foot_positions + LEG_OFFSETS_FROM_BODY_ORIGIN)  # Translate foot positions
