import math
import numpy as np
import numpy.typing as npt

from data_structures.gait_definition import LEG_COUNT

from interpolation import Interpolator, CatmullRomSpline


_GROUND_LEVEL = 0.0

class LegTrajectories:
    _phase_offset:     npt.NDArray[np.float64]  # (Time start offset); Out of 1
    _control_points:   npt.NDArray[np.float64]  # (Leg, Control Point, Coordinates) | Relative Coordinates

    foot_trajectories:       npt.NDArray[np.float64]  # (Leg, Knot/Point, Coordinates) | Relative Coordinates
    time_anchors:            npt.NDArray[np.float64]  # (Leg, time anchor)
    parametric_time_horizon: float  # The last parametric time entry

    steps_in_gait:    int
    distance_covered: float  # TODO: Implement


    def __init__(self, node_count:int,
                       leg_phase_offset:npt.NDArray[np.float64],
                       leg_control_points:npt.NDArray[np.float64]):
            # Shape check: (Leg)
            required_shape = (LEG_COUNT,)
            if leg_phase_offset.shape != required_shape:
                raise ValueError(f"Invalid shape {leg_phase_offset.shape}. Must be {required_shape}!")
            self.leg_phase_offset = leg_phase_offset
    
            # Shape check: (Leg, Control Point, Coordinates)
            if (leg_control_points.ndim != 3) or not (leg_control_points.shape[0] == LEG_COUNT and leg_control_points.shape[2] == 3):
                raise ValueError(f"Invalid shape {leg_control_points.shape}. Must be ({LEG_COUNT}, *, 3)!")
            self.leg_control_points = leg_control_points
    
            self.calculate_foot_trajectories(node_count)
    
    # TODO: Optimise
    def _calculate_step_count_and_time_horizon(self) -> None:
        for i in range(LEG_COUNT):
            last_time_anchor = self.time_anchors[i][-1]
            anchor_count     = len(self.time_anchors[i])
            phase_offset     = self._phase_offset[i]
            phase_multiplier = (1 - phase_offset)

            calculated_time = last_time_anchor * phase_multiplier
            
            if calculated_time > self.parametric_time_horizon:
                self.parametric_time_horizon = calculated_time
                self.steps_in_gait = math.ceil(anchor_count * phase_multiplier)

    def calculate_foot_trajectories(self, node_count:int) -> None:
        interpolator:Interpolator = CatmullRomSpline()

        self.foot_trajectories = np.empty((LEG_COUNT, node_count, 3), np.float64)  # (Leg, Control Point, Coordinates) | Relative Coordinates
        self.time_anchors      = np.empty((LEG_COUNT, node_count), np.float64)     # (Leg, time anchor)

        # TODO: Need to implement constant time, can just calculate last time anchor
        for i in range(LEG_COUNT):
            control_points = self._control_points[i]
            self.foot_trajectories[i], self.time_anchors[i] = interpolator.compute_interpolated_points(control_points, node_count)

        self._calculate_step_count_and_time_horizon()
