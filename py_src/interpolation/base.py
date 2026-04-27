import numpy as np
import numpy.typing as npt
from abc import ABC, abstractmethod
from data_structures.points import Point3D, PointList

class Interpolator(ABC):
    @abstractmethod
    def interpolate_point(self, t:float|np.float64, t_values:npt.NDArray[np.float64], control_points:PointList) -> Point3D:
        """
        Interpolates a single point at time t.

        Returns:
            Point3D: The interpolated point at time t.
        
        Args:
            t: The time-step we are evaluating.
            t_values: The array of time-anchors for each control point.
            control_points: The array of control points.
        """
        pass
