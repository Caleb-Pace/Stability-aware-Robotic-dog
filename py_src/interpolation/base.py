import numpy as np
import numpy.typing as npt
from abc import ABC, abstractmethod
from data_structures import Point3D, PointList

class Interpolator(ABC):
    @abstractmethod
    def interpolate_point(self, t:float, t_anchors:npt.NDArray[np.float64], control_points:PointList) -> Point3D:
        """
        Interpolates a single point at time t.

        Returns:
            Point3D: The interpolated point at time t.
        
        Args:
            t: The time-step we are evaluating.
            t_anchors: The array of time-anchors for each control point.
            control_points: The array of control points.
        """
        pass
    
    @abstractmethod
    def calculate_time_anchors(self, control_points:PointList) -> npt.NDArray[np.float64]:
        """
        Calculates the time-anchors for each control point.

        Returns:
            npt.NDArray[np.float64]: The array of time-anchors for each control point.

        Args:
            control_points: The array of control points.
        """
        pass

    def compute_interpolated_points(self, control_points:PointList, node_count:int, maximum_time:float|None=None) -> PointList:
        if node_count <= 0:
            raise ValueError("node_count must be positive")
        if len(control_points) < 2:
            raise IndexError(f"Not enough control points to interpolate! ({len(control_points)})")

        t_anchors = self.calculate_time_anchors(control_points)
        t_samples = np.linspace(0, (maximum_time or t_anchors[-1]), node_count)
    
        interpolated_points = np.empty((node_count, 3))  # Pre-allocate 2D array

        # Calculate interpolated points
        for i, t in enumerate(t_samples):
            if t > t_anchors[-1]:  # Sample not in range / on curve
                interpolated_points = interpolated_points[:i]  # Resize array to valid points
                break
            
            interpolated_points[i] = self.interpolate_point(t, t_anchors, control_points)

        return interpolated_points