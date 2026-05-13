import numpy as np
import numpy.typing as npt
from dataclasses import dataclass

@dataclass(frozen=True)
class Gait:
    body_height: npt.NDArray[np.float64]  # (Height)
    # body_pitch: float  # Future
    # body_roll: float  # Future

    leg_phase_offset:   npt.NDArray[np.float64]  # (Time start offset)
    leg_control_points: npt.NDArray[np.float64]  # (Leg, Control Point, Coordinate) | Relative Coordinates
    absolute_time:      float
