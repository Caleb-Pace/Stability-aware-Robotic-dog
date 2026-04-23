import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from visualisation.lagrange_interpolator import lagrange_interpolate

# Show plot
# TODO: Rename
def show_trajectory():
    ax = plt.figure().add_subplot(projection='3d')

    # Note: Linked to amount of points
    node_count = 16  # sample rate / accuracy
    t_sample = np.linspace(0, 1, node_count)  # Nodes

    control_points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 0, 2],
        [4, 0, 3],
        # [4.25, 0, 3.25],
        # [4.5, 0, 3.5],
        # [4.75, 0, 3.25],
        # [5, 0, 3]
    ])
    # TODO: Research more
    t_original = np.linspace(0, 1, len(control_points))  # Base t values

    interpolated_points = np.empty((node_count, 3))  # Pre-allocate 2D array
    for i, t in enumerate(t_sample):
        interpolated_points[i] = lagrange_interpolate(control_points, t_original, t)

    new_points_mask = ~np.isin(interpolated_points, control_points).all(axis=1)

    draw_parametric_function(ax, interpolated_points, '#f5d60a', 'Foot trajectory')
    draw_parametric_function(ax, control_points, "#56f43e87", 'Directly connection')
    plot_points(ax, interpolated_points[new_points_mask], '#00deff', 'Interpolated points')
    plot_points(ax, control_points, '#ff50a5', 'Control points')
    
    ax.legend()
    plt.show()

def draw_parametric_function(ax, points:npt.NDArray[np.float64], colour:str, label:str):
    tx = points[:, 0]
    ty = points[:, 1]
    tz = points[:, 2]

    # Draw curve
    ax.plot(tx, ty, tz, c=colour, label=label)

def plot_points(ax, points:npt.NDArray[np.float64], colour:str, label:str):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Plot points
    ax.scatter(x, y, z, c=colour, label=label)
