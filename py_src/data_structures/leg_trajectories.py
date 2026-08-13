import math
import numpy as np
import numpy.typing as npt

from data_structures.constants import LEG_COUNT

from interpolation import Interpolator, CatmullRomSpline


_GROUND_LEVEL = 0.0

class LegTrajectories:
    _phase_offset:   npt.NDArray[np.float64]  # Normalised [0.0, 1.0); Movement delay
    _control_points: npt.NDArray[np.object_]  # (Leg, Control Points, Relative Coordinates)

    foot_trajectories:       list  # (Leg, Knot/Point, Relative Coordinates)
    time_anchors:            list  # (Leg, time anchor)
    parametric_time_horizon: float  # The last parametric time entry

    steps_in_gait:    int
    distance_covered: float  # TODO: Implement


    def __init__(self, sample_count:int,
                       leg_phase_offset:npt.NDArray[np.float64],
                       leg_control_points:npt.NDArray[np.object_]):
        # Shape checks
        #     (Leg)
        required_shape = (LEG_COUNT,)
        if leg_phase_offset.shape != required_shape:
            raise ValueError(f"Invalid shape {leg_phase_offset.shape}. Must be {required_shape}!")
        
        #     (Leg, Control Point, Coordinates)
        if len(leg_control_points) != LEG_COUNT:
            raise ValueError(f"Expected {LEG_COUNT} legs, got {len(leg_control_points)}.")
        for i, leg in enumerate(leg_control_points):
            arr = np.asarray(leg)
            if arr.ndim != 2 or arr.shape[1] != 3:
                raise ValueError(f"Invalid shape {arr.shape} for leg {i}. Must be (*, 3)!")
            if len(arr) == 0:
                raise ValueError(f"Invalid point count 0 at leg {i}. Must be at least 1!")
        
        self._phase_offset         = leg_phase_offset
        self._control_points       = leg_control_points
    
        self.calculate_foot_trajectories(sample_count)

    def _calculate_last_time_anchor(self, leg:int, alpha:float = 0.5) -> float:
        """
        Calculates the final time anchor for a leg.

        Returns:
            npt.NDArray[np.float64]: Movement parametric duration.

        Args:
            alpha: Knot spacing parameter (e.g., 0.5 for centripetal).
        """
        if len(self._control_points[leg]) <= 1:
            return 0.0  # Early exit: hold point

        control_points = np.asarray(self._control_points[leg], dtype=np.float64)

        # Differences between consecutive control points
        diffs = np.diff(control_points, axis=0)
        
        # Euclidean distances between consecutive control points
        distances = np.linalg.norm(diffs, axis=-1)
        
        # Sum time calculation, per leg
        return float(np.sum(distances**alpha))

    def _calculate_time_horizon(self, alpha:float = 0.5) -> None:
        movement_horizons        = np.asarray([
            self._calculate_last_time_anchor(leg, alpha) for leg in range(LEG_COUNT)
        ])
        max_movement_horizon     = max(movement_horizons)

        normalized_phase_offsets = self._phase_offset % 1.0
        movement_delays          = normalized_phase_offsets * max_movement_horizon   # Parametric duration

        leg_durations = movement_delays + movement_horizons  # Parametric duration
        self.parametric_time_horizon = max(leg_durations)    # Parametric duration of the gait

    def calculate_foot_trajectories(self, sample_count:int) -> None:
        ALPHA = 0.5  # Centripedal knot spacing
        self.steps_in_gait = sample_count
        self._calculate_time_horizon(ALPHA)
        interpolator:Interpolator = CatmullRomSpline(ALPHA)

        self.foot_trajectories = [None] * LEG_COUNT
        self.time_anchors      = [None] * LEG_COUNT
        
        # Interpolate foot trajectiories with constant time
        for leg in range(LEG_COUNT):
            if len(self._control_points[leg]) == 1:  # Handle hold point
                self.foot_trajectories[leg] = self._control_points[leg][0]
                self.time_anchors[leg]      = 0.0
                continue

            # TODO: Remove, for debugging
            if len(self._control_points[leg]) > 3:
                print(f"[{leg}] {self._control_points[leg][0]} #{len(self._control_points[leg])}")

            self.foot_trajectories[leg], self.time_anchors[leg] = interpolator.compute_interpolated_points(
                                                                      self._control_points[leg],
                                                                      self.steps_in_gait,
                                                                      self.parametric_time_horizon
                                                                  )

        # TODO: Remove, for debugging
        for leg in range(LEG_COUNT):
            if len(self._control_points[leg]) > 3:
                print(f"[{leg}] {self._control_points[leg][0]} #{len(self._control_points[leg])}")
