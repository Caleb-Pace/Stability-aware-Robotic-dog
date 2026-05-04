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
        if node_count < 2:
            raise IndexError(f"Not enough nodes to interpolate! ({len(control_points)})")
        if len(control_points) < 2:
            raise IndexError(f"Not enough control points to interpolate! ({len(control_points)})")

        t_anchors = self.calculate_time_anchors(control_points)
        if len(control_points) != len(t_anchors):  # Sanity check
            raise IndexError(f"Bad {{Interpolator.calculate_time_anchors}} method! (Length mismatch between {{control_points}} and {{t_anchors}}! {len(control_points)} != {len(t_anchors)})")

        t_samples = np.linspace(0, (maximum_time or t_anchors[-1]), node_count)
    
        # Remove out of range time samples (samples that are not on the curve)
        if not maximum_time:  
            start_index_offset = 0
            end_index_offset = -1

            # Start - First sample on curve
            for i, t in enumerate(t_samples):
                if t >= t_anchors[0]:
                    start_index_offset = i
                    break

            # End - First sample after curve
            for i, t in enumerate(t_samples):
                if t > t_anchors[-1]:
                    end_index_offset = i - 1
                    break

            t_samples = t_samples[start_index_offset:end_index_offset]

        if len(t_samples) < 2:  # Sanity check
            raise IndexError(f"Not enough time samples to interpolate! ({len(t_samples)})")

        interpolated_points = np.empty((len(t_samples), 3))  # Pre-allocate PointList (2D array)

        # Calculate interpolated points
        for i, t in enumerate(t_samples):
            interpolated_points[i] = self.interpolate_point(t, t_anchors, control_points)

        return interpolated_points