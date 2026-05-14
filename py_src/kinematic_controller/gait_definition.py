import numpy as np
import numpy.typing as npt
from interpolation import Interpolator, CatmullRomSpline

LEG_COUNT:int = 4

class Gait:
    leg_phase_offset:   npt.NDArray[np.float64]  # (Time start offset)
    leg_control_points: npt.NDArray[np.float64]  # (Leg, Control Point, Coordinates) | Relative Coordinates
    foot_trajectories:  npt.NDArray[np.float64]  # (Leg, Knot/Point, Coordinates) | Relative Coordinates
    time_anchors:       npt.NDArray[np.float64]  # (Leg, time anchor)
    time_reference:     float = 0  # Calculated last time step

    body_height: npt.NDArray[np.float64]  # (Heights)
    # body_pitch: float  # Future
    # body_roll: float  # Future

    def __init__(self, node_count:int,
                 leg_phase_offset:npt.NDArray[np.float64],
                 leg_control_points:npt.NDArray[np.float64],
                 body_height:npt.NDArray[np.float64]):
        # Shape check: (Leg)
        required_shape = (LEG_COUNT,)
        if leg_phase_offset.shape != required_shape:
            raise ValueError(f"Invalid shape {leg_phase_offset.shape}. Must be {required_shape}!")
        self.leg_phase_offset = leg_phase_offset

        # Shape check: (Leg, Control Point, Coordinates)
        if (leg_control_points.ndim != 3) or not (leg_control_points.shape[0] == LEG_COUNT and leg_control_points.shape[2] == 3):
            raise ValueError(f"Invalid shape {leg_control_points.shape}. Must be ({LEG_COUNT}, *, 3)!")
        self.leg_control_points = leg_control_points

        # Shape check: (Heights)
        if body_height.ndim != 1:
            raise ValueError(f"Invalid shape {body_height.shape}. Must be (*,)!")
        self.body_height = body_height

        self.calculate_foot_trajectories(node_count)
        self._calculate_time_reference()

    def calculate_foot_trajectories(self, node_count:int) -> None:
        interpolator:Interpolator = CatmullRomSpline()

        self.foot_trajectories = np.empty((LEG_COUNT, node_count, 3), np.float64)  # (Leg, Control Point, Coordinates) | Relative Coordinates
        self.time_anchors      = np.empty((LEG_COUNT, node_count), np.float64)     # (Leg, time anchor)

        for i in range(LEG_COUNT):
            control_points = self.leg_control_points[i]
            self.foot_trajectories[i], self.time_anchors[i] = interpolator.compute_interpolated_points(control_points, node_count)
    
    # TODO: Optimise
    def _calculate_time_reference(self) -> None:
        for anchor, phase_offset in zip(self.time_anchors[:LEG_COUNT, -1], self.leg_phase_offset[:LEG_COUNT]):
            calculated_time = anchor * (phase_offset + 1)
            
            if calculated_time > self.time_reference:
                self.time_reference = calculated_time
