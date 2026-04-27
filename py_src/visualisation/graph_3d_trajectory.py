import numpy as np
import matplotlib.pyplot as plt
from visualisation.lagrange_interpolator import lagrange_interpolate, PointList

# Show plot
# TODO: Rename
def show_trajectory():
    ax = plt.figure().add_subplot(projection='3d')

    # Note: Linked to amount of points
    node_count = 32  # sample rate / accuracy
    t_samples = np.linspace(0, 1, node_count)  # Nodes
    print(repr(t_samples))

    control_points = np.array([
        [0, 0, 0],
        [1, 1, 1],
        [2, 0, 2],
        [4, 0, 3],
        # [4.25, 0, 3.25],
        # [4.5, 0, 3.5],
        # [4.75, 0, 3.25],
        [5, 0, 3],
        [4, 0, 3],
        [2, 2, 2],
        [3, 2, 3],
        [4, 3, 3],
        [3.5, 3, 2.5]
    ])
    # TODO: Research more
    t_original = np.linspace(0, 1, len(control_points))  # Base t values

    interpolated_points = np.empty((node_count, 3))  # Pre-allocate 2D array
    for i, t in enumerate(t_samples):
        interpolated_points[i] = lagrange_interpolate(t, t_original, control_points)

    new_points_mask = ~np.isin(interpolated_points, control_points).all(axis=1)

    draw_parametric_function(ax, interpolated_points, '#f5d60a', 'Foot trajectory')
    plot_points(ax, interpolated_points[new_points_mask], '#00deff', 'Interpolated points')
    plot_points(ax, control_points, '#ff50a5', 'Control points')
    
    ax.legend()
    plt.show()

def draw_parametric_function(ax, points:PointList, colour:str, label:str):
    tx = points[:, 0]
    ty = points[:, 1]
    tz = points[:, 2]

    # Draw curve
    ax.plot(tx, ty, tz, c=colour, label=label)

def plot_points(ax, points:PointList, colour:str, label:str):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Plot points
    ax.scatter(x, y, z, c=colour, label=label)
