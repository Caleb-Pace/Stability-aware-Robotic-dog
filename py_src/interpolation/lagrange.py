import numpy as np
import numpy.typing as npt
from data_structures.points import Point3D, PointList

def lagrange_basis(i:int, t:float|np.float64, t_values:npt.NDArray[np.float64]) -> float:
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

def lagrange_interpolate(t:float|np.float64, t_values:npt.NDArray[np.float64], control_points:PointList) -> Point3D:        
    """
    Interpolates a single point at time t.

    Returns:
        Point3D: The interpolated point at time t.
    
    Args:
        t: The time-step we are evaluating.
        t_values: The array of time-anchors for each control point.
        control_points: The array of control points.
    """
    # Generate weights for all control points
    #   the influence each control point has on the curve (at time t)
    weights = np.array([lagrange_basis(i, t, t_values) for i in range(len(control_points))])

    # Calculate point based on weights, using dot product
    return np.dot(weights, control_points)
