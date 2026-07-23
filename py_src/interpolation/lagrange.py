import numpy as np
import numpy.typing as npt
from data_structures.points import Point3D, Point3DList
from typing import override
from . import Interpolator

class Lagrange(Interpolator):
    def lagrange_basis(self, i:int, t:float, t_values:npt.NDArray[np.float64]) -> float:
        """
        Compute the i-th Lagrange basis polynomial weight at time t.

        Note:
            Assumes t_values are unique to avoid division by zero.
        
        Returns:
            (float): The influence (Lagrange basis polynomial weight) the i-th control point has on the curve at time t.
        
        Args:
            i: The control point index we are calculating weight for.
            t: The time-step we are evaluating.
            t_values: The array of time-anchors for each control point.
        """
        basis = 1.0

        for j, t_j in enumerate(t_values):
            if j != i:
                basis *= (t - t_j) / (t_values[i] - t_j)
        
        return basis

    @override
    def interpolate_point(self, t:float, t_anchors:npt.NDArray[np.float64], control_points:Point3DList) -> Point3D:
        # Generate weights for all control points
        #   the influence each control point has on the curve (at time t)
        weights = np.array([self.lagrange_basis(i, t, t_anchors) for i in range(len(control_points))])

        # Calculate point based on weights, using dot product
        return np.dot(weights, control_points)

    @override
    def calculate_time_anchors(self, control_points:Point3DList) -> npt.NDArray[np.float64]:
        return np.linspace(0, 1, len(control_points))