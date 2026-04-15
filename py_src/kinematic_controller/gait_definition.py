import numpy as np
import numpy.typing as npt
from dataclasses import dataclass, field

@dataclass(frozen=True)
class Gait:
    body_height: float
    # body_pitch: float  # Future
    # body_roll: float  # Future

    leg_phases_offset: npt.NDArray[np.float64]  # TODO: Could rename to leg_sequence
    leg_duty_factor: npt.NDArray[np.float64]
    duty_cycle_length: float
    
    # leg_start_position: np.ndarray = field(default_factory=lambda: np.zeros((4, 2), dtype=float))
    # leg_end_position: np.ndarray = field(default_factory=lambda: np.zeros((4, 2), dtype=float))
