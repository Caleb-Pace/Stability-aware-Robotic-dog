import math
import numpy as np
import numpy.typing as npt

from data_structures.gait_definition import LEG_COUNT

from interpolation import Interpolator, CatmullRomSpline


_GROUND_LEVEL = 0.0

class LegTrajectories:
    _phase_offset:   npt.NDArray[np.float64]  # Normalised [0.0, 1.0); Movement delay
    _control_points: npt.NDArray[np.float64]  # (Leg, Control Points, Relative Coordinates)

    foot_trajectories:       npt.NDArray[np.float64]  # (Leg, Knot/Point, Relative Coordinates)
    time_anchors:            npt.NDArray[np.float64]  # (Leg, time anchor)
    parametric_time_horizon: float  # The last parametric time entry

    steps_in_gait:    int
    distance_covered: float  # TODO: Implement


    def __init__(self, sample_count:int,
                       leg_phase_offset:npt.NDArray[np.float64],
                       leg_control_points:npt.NDArray[np.float64]):
        # Shape checks
        #     (Leg)
        required_shape = (LEG_COUNT,)
        if leg_phase_offset.shape != required_shape:
            raise ValueError(f"Invalid shape {leg_phase_offset.shape}. Must be {required_shape}!")
        
        #     (Leg, Control Point, Coordinates)
        if (leg_control_points.ndim != 3) or not (leg_control_points.shape[0] == LEG_COUNT and leg_control_points.shape[2] == 3):
                raise ValueError(f"Invalid shape {leg_control_points.shape}. Must be ({LEG_COUNT}, *, 3)!")

        self._phase_offset         = leg_phase_offset
        self._control_points       = leg_control_points
    
        self.calculate_foot_trajectories(sample_count)

    def _find_max_movement_horizon(self) -> float:
        """Finds the maximum movement parametric duration among all the legs"""
        return float(np.max(self.time_anchors[:, -1]))

    def _calculate_last_time_anchors(self, alpha:float = 0.5) -> npt.NDArray[np.float64]:
        """
        Calculates the final time anchor for each leg.

        Returns:
            npt.NDArray[np.float64]: Movement parametric duration per leg.

        Args:
            alpha: Knot spacing parameter (e.g., 0.5 for centripetal).
        """
        # Differences between consecutive control points
        diffs = np.diff(self._control_points, axis=1)
        
        # Euclidean distances between consecutive control points
        distances = np.linalg.norm(diffs, axis=-1)
        
        # Sum time calculation, per leg
        return np.sum(distances**alpha, axis=1)

    def _calculate_time_horizon(self) -> None:
        self.parametric_time_horizon = 0.0  # Clear previous value
        max_movement_horizon         = self._find_max_movement_horizon()

        movement_horizons        = self._calculate_last_time_anchors()   # Parametric duration
        normalized_phase_offsets = self._phase_offset % 1.0
        movement_delays          = normalized_phase_offsets * max_movement_horizon   # Parametric duration

        leg_durations = movement_delays + movement_horizons   # Parametric duration
        self.parametric_time_horizon = np.max(leg_durations)  # Parametric duration of the gait

    # TODO: fix, incorrect implementation, generates too many points?
    def calculate_foot_trajectories(self, sample_count:int) -> None:
        self.steps_in_gait = sample_count
        interpolator:Interpolator  = CatmullRomSpline()

        self.foot_trajectories = np.empty((LEG_COUNT, self.steps_in_gait, 3), np.float64)  # (Leg, Control Point, 3D Point)
        self.time_anchors      = np.empty((LEG_COUNT, self.steps_in_gait), np.float64)     # (Leg, time anchor)

        # TODO: Need to implement constant time, can just calculate last time anchor
        for leg in range(LEG_COUNT):
            self.foot_trajectories[leg], self.time_anchors[leg] = interpolator.compute_interpolated_points(
                                                                      self._control_points[leg],
                                                                      self.steps_in_gait
                                                                  )

        self._calculate_time_horizon()
