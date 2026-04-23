import numpy as np
import numpy.typing as npt
from typing import Annotated

type Point3D = Annotated[npt.NDArray[np.float64], (3,)]  # (A Coordinate) 1D array with 3 elements
type PointList = Annotated[npt.NDArray[np.float64], (None, 3)]  # 2D array of points/coordinates

def lagrange_basis(i:int, t:np.float64|float, t_values:npt.NDArray[np.float64]) -> float:
    """
    Compute the i-th Lagrange basis polynomial weight at time t.
    
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

def lagrange_interpolate(t:np.float64|float, t_values:npt.NDArray[np.float64], control_points:PointList) -> Point3D:
    """
    Interpolates a single point at time t.
    
    Args:
        t: The time-step we are evaluating.
        t_values: The array of time-anchors for each control point.
        control_points: The array of control points.
    """
    x, y, z = 0.0, 0.0, 0.0

    # Combine lagrange polynomials
    for i in range(len(control_points)):
        weight = lagrange_basis(i, t, t_values)
        x += control_points[i, 0] * weight
        y += control_points[i, 1] * weight
        z += control_points[i, 2] * weight
    
    return np.array([x, y, z])
