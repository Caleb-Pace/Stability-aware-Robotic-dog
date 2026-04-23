import numpy as np
import numpy.typing as npt

# TODO: Make clearer comments
def lagrange_basis(t, t_values, i):
    """Compute the i-th Lagrange basis polynomial at t."""
    """0 at all other points except i"""
    basis = 1.0

    for j, t_j in enumerate(t_values):
        if j != i:
            basis *= (t - t_j) / (t_values[i] - t_j)
    
    return basis

def lagrange_interpolate(points, t_values, t):
    """Interpolate 3D points at parameter t using Lagrange interpolation."""
    x, y, z = 0.0, 0.0, 0.0

    # Combine lagrange polynomials
    for i in range(len(points)):
        Li = lagrange_basis(t, t_values, i)
        x += points[i, 0] * Li
        y += points[i, 1] * Li
        z += points[i, 2] * Li
    
    return np.array([x, y, z])


def test():
    # Example: 3 points
    points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 0, 2]
    ])

    # Evenly spaced t_values
    t_values = np.linspace(0, 1, len(points))

    # Interpolate at t = 0.25, 0.5, 0.75
    t_test = [0.25, 0.5, 0.75]
    for t in t_test:
        print(f"Interpolated point at t={t}: {lagrange_interpolate(points, t_values, t)}")