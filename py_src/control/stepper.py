import numpy as np

from data_structures import Point3DList
from data_structures.leg_trajectories import LegTrajectories
from data_structures.gait_definition import LEG_COUNT

from kinematics.ik_solver import LEG_OFFSETS_FROM_BODY_ORIGIN


def step(trajectory:LegTrajectories, step_num:int) -> Point3DList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    for leg in range(LEG_COUNT):
        offset = trajectory.phase_offsets_as_steps[leg]

        # Calculate new point/step index
        index = offset + step_num
        if index > trajectory.steps_in_gait:  # Wrap index if necessary
            index %= trajectory.steps_in_gait

        foot_positions[leg] = trajectory.foot_trajectories[leg][index]
    
    return foot_positions



def apply_offset(foot_positions:Point3DList) -> Point3DList:
    if len(foot_positions) != LEG_COUNT:
        raise ValueError(f"{{foot_positions}} must be equal to the {{LEG_COUNT}}! ({foot_positions} == {LEG_COUNT})")
    
    return (foot_positions + LEG_OFFSETS_FROM_BODY_ORIGIN)  # Translate foot positions
