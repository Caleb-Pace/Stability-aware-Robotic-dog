import numpy as np
import numpy.typing as npt

from data_structures.constants import LEG_COUNT, JOINT_COUNT  # Related constants
from data_structures.leg_trajectories import LegTrajectories


class Gait:
    transition_in:LegTrajectories|None
    transition_out:LegTrajectories|None
    loop:LegTrajectories


    def create_transitions(self):
        self.loop.phase_offsets_as_steps

        if sum(self.loop.phase_offsets_as_steps) == 0:
            self.transition_in = None
            self.transition_out = None
            return  # Early exit: no transition needed.

        # Note: Extract Transition in
        points_per_leg = []
        for leg in range(LEG_COUNT):
            leg_steps       = self.loop.foot_trajectories[leg]
            available_steps = len(leg_steps)
            offset          = self.loop.phase_offsets_as_steps[leg]

            # TODO: Raise exception when offset is past available_steps
            
            start = offset
            end   = start + (offset - 1)

            # Extract points
            leg_points = []
            if end < available_steps:
                leg_points = leg_steps[start:end]
            else:
                leg_points = leg_steps[start:available_steps]

                remaining   = end - available_steps
                leg_points += leg_steps[0:remaining]

            points_per_leg.append( np.asarray(leg_points) )

        self.transition_in = LegTrajectories(
            sample_count       = len(points_per_leg),
            heights            = np.array([0.3], dtype=float),        # TODO: Implement (Extract current)
            orientations       = np.array([[0, 0, 0]], dtype=float),  # TODO: Implement (Extract with control points)
            leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
            leg_control_points = np.asarray( points_per_leg ),
            should_interpolate = False
        )
        self.transition_in.time_anchors = []

        for i in range(LEG_COUNT):
            points_per_leg[i] = points_per_leg[i][::-1, :]  # Reverse

        # Note: Reverse Transition in to get Transition out
        self.transition_out = LegTrajectories(
            sample_count       = len(points_per_leg),
            heights            = np.array([0.3], dtype=float),        # TODO: Implement (Extract current)
            orientations       = np.array([[0, 0, 0]], dtype=float),  # TODO: Implement (Extract with control points)
            leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
            leg_control_points = np.asarray( points_per_leg ),
            should_interpolate = False
        )
        self.transition_out.time_anchors = []

    def __init__(self, loop:LegTrajectories):
        self.loop = loop

        self.create_transitions()
