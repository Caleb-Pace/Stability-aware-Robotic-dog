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
        """
        Computes a list of interpolated points along the curve defined by the control points.

        Returns:
            PointList: The array of interpolated points along the curve.

        Args:
            control_points: The array of control points defining the curve.
            node_count: The number of points to interpolate along the curve.
            maximum_time: The maximum time-step to evaluate up to (for absolute time).
                Defaults to the last time-anchor if not provided (relative time).

        Raises:
            IndexError: If fewer than 2 control points are provided.
            ValueError: If node_count is not positive.
            ValueError: If calculate_time_anchors is incorrectly implemented in a subclass.
            IndexError: If calculate_time_anchors is incorrectly implemented in a subclass.
        """
        
        if len(control_points) < 2:
            raise IndexError(f"Not enough control points to interpolate! ({len(control_points)} >= 2)")
        
        if node_count <= 0:
            raise ValueError(f"{{node_count}} must be positive! ({node_count} > 0)")


        t_anchors = self.calculate_time_anchors(control_points)
        if not np.all(np.diff(t_anchors) > 0):     # Sanity check - Strictly increasing (no duplicates)
            raise ValueError(f"Bad {{{self.__class__.__name__}.calculate_time_anchors}} method!\n\t{{t_anchors}} must be strictly increasing.")
        if t_anchors[0] != 0:                      # Sanity check - Starts at 0
            raise ValueError(f"Bad {{{self.__class__.__name__}.calculate_time_anchors}} method!\n\tCurve origin/{{t_anchors[0]}} must be zero! ({t_anchors[0]} == 0)")
        if len(control_points) != len(t_anchors):  # Sanity check - Corresponds to control points
            raise IndexError(f"Bad {{{self.__class__.__name__}.calculate_time_anchors}} method!\n\tLength mismatch between {{control_points}} and {{t_anchors}}. ({len(control_points)} == {len(t_anchors)})")


        t_samples = np.linspace(0, (maximum_time or t_anchors[-1]), node_count)  # Evenly space samples
    
        # Remove out of range time samples (samples that are not on the curve)
        if maximum_time:
            end_index_offset = len(t_samples)

            # End - First sample after curve
            for i, t in enumerate(t_samples):
                if t > t_anchors[-1]:
                    end_index_offset = i - 1
                    break

            t_samples = t_samples[:end_index_offset]

        if len(t_samples) <= 0:  # Sanity check
            raise IndexError(f"Not enough time samples to interpolate! ({len(t_samples)} > 0)")


        interpolated_points = np.empty((len(t_samples), 3))  # Pre-allocate PointList (2D array)

        # Calculate interpolated points
        for i, t in enumerate(t_samples):
            interpolated_points[i] = self.interpolate_point(t, t_anchors, control_points)

        return interpolated_points
