import numpy as np

from data_structures.constants import LEG_COUNT
from data_structures.leg_trajectories import LegTrajectories


class Gait:
    transition_in: LegTrajectories|None
    transition_out:LegTrajectories|None
    loop:LegTrajectories


    def create_transitions(self):
        self.loop.phase_offsets_as_steps

        if sum(self.loop.phase_offsets_as_steps) == 0:
            self.transition_in = None
            self.transition_out = None
            return  # Early exit: no transition needed.

        max_offset = max(self.loop.phase_offsets_as_steps)
        print(f"max:{max_offset} | {self.loop.phase_offsets_as_steps}")

        # Note: Extract Transition in
        points_per_leg  = []
        for leg in range(LEG_COUNT):
            leg_steps        = self.loop.foot_trajectories[leg]
            available_steps  = len(leg_steps)
            offset           = self.loop.phase_offsets_as_steps[leg]

            # TODO: Raise exception when offset is past available_steps
            
            if offset == 0:
                n = max_offset + 1
                points_per_leg.append( np.array([leg_steps[0]] * n) )
                continue

            end = offset

            # Extract points
            leg_points  = []
            if end < available_steps:
                leg_points = []
                leg_points.extend( [leg_steps[0]] * (max_offset - offset + 1) )
                leg_points.extend( leg_steps[1:end + 1] )
            else:
                # TODO: Implement
                print("Not implemented yet... womp womp")
                import sys
                sys.exit(1)

            points_per_leg.append( np.asarray(leg_points) )

        leg_origins = self.loop.leg_origins_by_step[0:max_offset + 1]

        self.transition_in = LegTrajectories(
            sample_count       = max_offset + 1,
            heights            = np.array([0.3], dtype=float),        # TODO: Implement (Extract current)
            orientations       = np.array([[0, 0, 0]], dtype=float),  # TODO: Implement (Extract with control points)
            leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
            leg_control_points = np.asarray( points_per_leg, dtype=object ),
            should_interpolate = False
        )
        self.transition_in.leg_origins_by_step = leg_origins


        # Note: Reverse Transition in to get Transition out
        for i in range(LEG_COUNT):
            points_per_leg[i] = points_per_leg[i][::-1, :][1:]  # Reverse and skip first

        self.transition_out = LegTrajectories(
            sample_count       = max_offset,
            heights            = np.array([0.3], dtype=float),        # TODO: Implement (Extract current)
            orientations       = np.array([[0, 0, 0]], dtype=float),  # TODO: Implement (Extract with control points)
            leg_phase_offset   = np.array([0, 0, 0, 0], dtype=float),
            leg_control_points = np.asarray( points_per_leg, dtype=object ),
            should_interpolate = False
        )
        self.transition_out.leg_origins_by_step = leg_origins[::-1][1:]  # Reverse and skip first

    def __init__(self, loop:LegTrajectories):
        self.loop = loop

        self.create_transitions()
