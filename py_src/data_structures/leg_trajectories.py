import numpy as np
import numpy.typing as npt

from data_structures import Point3D, EulerAngles, Point3DList

from data_structures.constants import LEG_COUNT
from kinematics.ik_solver import LEG_OFFSETS_FROM_BODY_ORIGIN

from interpolation import Interpolator, CatmullRomSpline


_GROUND_LEVEL = -0.3

class LegTrajectories:
    _body_heights:   npt.NDArray[np.float64]  # z values
    _orientations:   npt.NDArray[np.float64]  # (Control Points, Euler angles)
    _control_points: npt.NDArray[np.object_]  # (Leg, Control Points, Relative Coordinates)
    _phase_offset:   npt.NDArray[np.float64]  # Normalised [0.0, 1.0); Movement delay

    leg_origins:             list   # (Leg, Knot/Point, Relative Coordinates)
    foot_trajectories:       list   # (Leg, Knot/Point, Relative Coordinates)
    time_anchors:            list   # (Leg, time anchor)
    parametric_time_horizon: float  # The last parametric time entry

    steps_in_gait:    int
    distance_covered: float  # TODO: Implement


    def __init__(self, sample_count:int,
                       heights:npt.NDArray[np.float64],
                       orientations:npt.NDArray[np.float64],
                       leg_phase_offset:npt.NDArray[np.float64],
                       leg_control_points:npt.NDArray[np.object_]):
        # Shape checks
        #     (Leg)
        required_shape = (LEG_COUNT,)
        if leg_phase_offset.shape != required_shape:
            raise ValueError(f"Invalid shape {leg_phase_offset.shape}. Must be {required_shape}!")
        if heights.shape != required_shape:
            raise ValueError(f"Invalid shape {heights.shape}. Must be {required_shape}!")
        
        #     (Control Points, Euler angles)
        required_shape = (None,3)
        if orientations.shape != required_shape:
            raise ValueError(f"Invalid shape {orientations.shape}. Must be {required_shape}!")
        
        #     (Leg, Control Point, Coordinates)
        if len(leg_control_points) != LEG_COUNT:
            raise ValueError(f"Expected {LEG_COUNT} legs, got {len(leg_control_points)}.")
        for i, leg in enumerate(leg_control_points):
            arr = np.asarray(leg)
            if arr.ndim != 2 or arr.shape[1] != 3:
                raise ValueError(f"Invalid shape {arr.shape} for leg {i}. Must be (*, 3)!")
            if len(arr) == 0:
                raise ValueError(f"Invalid point count 0 at leg {i}. Must be at least 1!")

        self._body_heights         = heights
        self._orientations         = orientations
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

    def _calculate_distance_between_steps(self, leg:int, start:int|None = None, end:int|None = None) -> float:
        if start is None:
            start = 0
        if end is None:
            end = len(self.foot_trajectories[leg])
        points = np.asarray( self.foot_trajectories[leg][start:end] )

        # Calculate distance moved on ground
        distance_moved:float = 0.0
        for p1, p2 in zip(points[:-1], points[1:]):

            if p1[2] <= _GROUND_LEVEL and p2[2] <= _GROUND_LEVEL:
                distance_moved += float( p2[0] - p1[0] )  # By change in x

        return -distance_moved

    def _calculate_distance(self) -> None:
        distance_covered_per_leg = [ self._calculate_distance_between_steps(i) for i in range(LEG_COUNT) ]
        self.distance_covered = max(distance_covered_per_leg)

    def _interpolate(self, interpolator:Interpolator, data:Point3DList, sample_count:int|None = None) -> tuple[Point3DList, npt.NDArray[np.float64]]:
        if len(data) == 1:  # Handle hold point
           point:Point3DList = np.array([ data[0] ], dtype=np.float64)
           time_anchor       = np.array([ 0.0 ], dtype=float)
           return ( point, time_anchor )  # Early exit: only 1 point

        if sample_count == None:
            sample_count = self.steps_in_gait

        return interpolator.compute_interpolated_points(data, sample_count, self.parametric_time_horizon)

    # def _apply_orientation(self, points:Point3DList, orientation:EulerAngles) -> npt.NDArray[np.float64]:
    #     """
    #     points: (4, 3) array of coordinates
    #     orientation_rpy: [roll, pitch, yaw] in radians
    #     """
    #     roll, pitch, yaw = orientation
    #     cr, sr = np.cos(roll),  np.sin(roll)
    #     cp, sp = np.cos(pitch), np.sin(pitch)
    #     cy, sy = np.cos(yaw),   np.sin(yaw)

    #     # Composite Z-Y-X rotation matrix  (Aerospace standard)
    #     R = np.array([
    #         [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
    #         [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
    #         [  -sp,             cp*sr,             cp*cr]
    #     ])

    #     # Apply to all points at once: (4, 3) @ (3, 3) -> (4, 3)
    #     return np.asarray(points) @ R.T

    # def _calculate_leg_origins(self, body_height:float, orientation:EulerAngles) -> npt.NDArray[np.float64]:
    #     leg_origins:Point3DList = LEG_OFFSETS_FROM_BODY_ORIGIN
    #     leg_origins[:, 1]       = body_height

    #     return self._apply_orientation(leg_origins, orientation)

    # # TODO: Rename
    # def _calculate_origins_over_time(self, interpolator:Interpolator) -> None:
    #     leg_origins = np.empty((LEG_COUNT, len(self._body_heights), 3))
    #     leg_origins[:, :, :2] = LEG_OFFSETS_FROM_BODY_ORIGIN[:, np.newaxis, :]
    #     leg_origins[:, :, 2]  = self._body_heights

    #     interpolated_body_heights = self._interpolate(interpolator, leg_origins)
    #     interpolated_orientations = self._interpolate(interpolator, self._orientations)

    #     self.leg_origins

    #     # for 
    #     #     self._calculate_leg_origins()

    # TODO: Rename
    def calculate_foot_trajectories(self, sample_count:int) -> None:
        ALPHA = 0.5  # Centripedal knot spacing
        self.steps_in_gait = sample_count
        self._calculate_time_horizon(ALPHA)
        interpolator:Interpolator = CatmullRomSpline(ALPHA)

        # Find leg origins for each time anchor
        # self._calculate_leg_origins(interpolator)

        # Interpolate foot trajectiories with constant time
        self.foot_trajectories = [None] * LEG_COUNT
        self.time_anchors      = [None] * LEG_COUNT

        for leg in range(LEG_COUNT):
            data = self._control_points[leg]

            self.foot_trajectories[leg], self.time_anchors[leg] = self._interpolate(interpolator, data)

        # Find travel distance
        self._calculate_distance()
