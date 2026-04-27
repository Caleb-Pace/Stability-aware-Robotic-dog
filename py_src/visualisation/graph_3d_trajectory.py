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

    ctrl_pts_base = np.array([
        [-9.0, -2.5, -8.5],
        [-8.4, -6.8, -5.1],
        [-4.5, -8.2, -4.2],
        [-2.1, -4.9, -1.8],
        [ 1.9, -2.4,  1.2],
        [ 4.1,  3.1,  1.7],
        [ 1.2,  8.0,  3.9]
    ])
    ctrl_pts_set_a = np.array([
        [ 2.8,  6.4,  7.8],
        [ 7.2,  4.9,  9.3],
        [ 9.4,  0.5,  8.1],
        [ 8.7, -4.1,  8.8],
        [ 6.5, -7.9,  4.2],
        [ 3.1, -9.2,  2.8]
    ])
    ctrl_pts_set_b = np.array([
        [ 1.2,  8.0,  3.9],  # Anchor: Matches end of ctrl_pts_base
        [-1.5,  8.5,  5.0],  # Smooth transition heading toward -X
        [-4.0,  7.0,  7.5],  # Climbing and curving
        [-6.5,  4.0,  9.0],  # Maintaining height while moving away
        [-8.0,  0.0,  8.5],  # Curving back down the X-axis
        [-7.0, -4.0,  6.0]   # Final point tucked into the -X, -Y, +Z quadrant
    ])
    control_points = np.concatenate((ctrl_pts_base, ctrl_pts_set_a))
    control_points_alt = np.concatenate((ctrl_pts_base, ctrl_pts_set_b))

    # TODO: Research more
    t_original = np.linspace(0, 1, len(control_points))  # Base t values

    interpolated_points = np.empty((node_count, 3))  # Pre-allocate 2D array
    interpolated_points_alt = np.empty((node_count, 3))  # Pre-allocate 2D array
    for i, t in enumerate(t_samples):
        interpolated_points[i] = lagrange_interpolate(t, t_original, control_points)
        interpolated_points_alt[i] = lagrange_interpolate(t, t_original, control_points_alt)

    ## Original path
    new_points_mask = ~np.isin(interpolated_points, control_points).all(axis=1)
    draw_parametric_function(ax, interpolated_points, '#FF8C00', 'Foot trajectory')
    draw_parametric_function(ax, control_points, "#8B000090", 'Directly connection')
    plot_points(ax, interpolated_points[new_points_mask], '#FF7F50', 'Interpolated points')
    plot_points(ax, control_points, '#BC8F8F', 'Control points')

    ## Path switch
    new_points_mask = ~np.isin(interpolated_points_alt, np.concatenate((control_points, interpolated_points))).all(axis=1)
    draw_parametric_function(ax, interpolated_points_alt, "#003366", 'Foot trajectory Alt')
    draw_parametric_function(ax, control_points_alt, "#55555590", 'Directly connection Alt')
    plot_points(ax, interpolated_points_alt[new_points_mask], '#4682B4', 'Interpolated points Alt')
    plot_points(ax, ctrl_pts_set_b, "#A9A9A9", 'Control points Alt')
    
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
