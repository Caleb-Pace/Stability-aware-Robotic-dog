import numpy as np

from data_structures import Point3DList
from data_structures.leg_trajectories import LegTrajectories
from data_structures.gait_definition import LEG_COUNT

from kinematics.ik_solver import LEG_OFFSETS_FROM_BODY_ORIGIN


def step(trajectory:LegTrajectories, step_num:int) -> Point3DList:
    foot_positions = np.zeros((LEG_COUNT, 3), dtype=float)

    for leg in range(LEG_COUNT):
        available_steps = len(trajectory.foot_trajectories[leg])

        offset = trajectory.phase_offsets_as_steps[leg]

        # Calculate new point/step index
        index = offset + step_num
        if index >= available_steps:  # Wrap index if necessary
            index %= available_steps

        print(f"Index: {index} ( ({offset} + {step_num}) % aS#{available_steps}); gS#{trajectory.steps_in_gait}; {len(trajectory.foot_trajectories[leg])}")
        foot_positions[leg] = trajectory.foot_trajectories[leg][index]
    
    return foot_positions



def apply_offset(foot_positions:Point3DList) -> Point3DList:
    if len(foot_positions) != LEG_COUNT:
        raise ValueError(f"{{foot_positions}} must be equal to the {{LEG_COUNT}}! ({foot_positions} == {LEG_COUNT})")
    
    return (foot_positions + LEG_OFFSETS_FROM_BODY_ORIGIN)  # Translate foot positions
