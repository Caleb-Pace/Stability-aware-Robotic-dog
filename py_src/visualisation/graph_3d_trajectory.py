import numpy as np
import matplotlib.pyplot as plt
import interpolation
from interpolation import Interpolator
from data_structures import PointList

# Show plot
# TODO: Rename
def show_trajectory(interpolator:Interpolator, alt_interpolator:Interpolator|None = None):
    ax = plt.figure().add_subplot(projection='3d')

    knots_base = np.array([
        [-9.0, -2.5, -8.5],
        [-8.4, -6.8, -5.1],
        [-4.5, -8.2, -4.2],
        [-2.1, -4.9, -1.8],
        # [ 1.9, -2.4,  1.2],
        # [ 4.1,  3.1,  1.7],
        # [ 1.2,  8.0,  3.9]
    ])
    knots_set_a = np.array([
        [ 2.8,  6.4,  7.8],
        [ 7.2,  4.9,  9.3],
        [ 9.4,  0.5,  8.1],
        [ 8.7, -4.1,  8.8],
        [ 6.5, -7.9,  4.2],
        [ 3.1, -9.2,  2.8]
    ])
    knots_set_b = np.array([
        [ 1.2,  8.0,  3.9],  # Anchor: Matches end of ctrl_pts_base
        [-1.5,  8.5,  5.0],  # Smooth transition heading toward -X
        [-4.0,  7.0,  7.5],  # Climbing and curving
        [-6.5,  4.0,  9.0],  # Maintaining height while moving away
        [-8.0,  0.0,  8.5],  # Curving back down the X-axis
        [-7.0, -4.0,  6.0]   # Final point tucked into the -X, -Y, +Z quadrant
    ])
    knots = np.concatenate((knots_base, knots_set_a))
    knots_alt = np.concatenate((knots_base, knots_set_b))

    # Note: Linked to amount of points
    node_count = 12  # sample rate / accuracy
    time_limit = 25  # Common limit for absolute time

    ## Original path
    draw_parametric_function(ax, knots, "#8B000090", 'Directly connection')
    plot_points(ax, knots, '#BC8F8F', 'Control points')
    draw_interpolated_curve(ax, interpolator, knots, node_count, 'Foot trajectory', '#FF8C00', '#FF7F50', time_limit)

    ## Path switch
    if alt_interpolator != None:
        draw_parametric_function(ax, knots_alt, "#55555590", 'Directly connection Alt')
        plot_points(ax, knots_set_b, "#A9A9A9", 'Control points Alt')
        draw_interpolated_curve(ax, alt_interpolator, knots_alt, node_count, 'Foot trajectory Alt', '#003366', '#4682B4', time_limit)
    
    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

def compare_interpolators():
    ax = plt.figure().add_subplot(projection='3d')

    points = np.array([
        [-9.0, -2.5, -8.5],
        [-8.4, -6.8, -5.1],
        [-4.5, -8.2, -4.2],
        [-2.1, -4.9, -1.8],
    ])

    node_count = 12  # sample rate / accuracy

    draw_parametric_function(ax, points, "#91919190", 'Direct connection')
    plot_points(ax, points, "#5A5A5A", 'Points')
    
    draw_interpolated_curve(ax, interpolation.Lagrange(), points, node_count, 'Lagrange', '#873BB3', '#7723A7')
    draw_interpolated_curve(ax, interpolation.CatmullRomSpline(alpha=0.0), points, node_count, 'Catmull-Rom | α=0.0', '#D8379A', '#CC1A88')
    draw_interpolated_curve(ax, interpolation.CatmullRomSpline(alpha=0.5), points, node_count, 'Catmull-Rom | α=0.5', '#FF5879', '#FF5879')
    draw_interpolated_curve(ax, interpolation.CatmullRomSpline(alpha=1.0), points, node_count, 'Catmull-Rom | α=1.0', '#FF8D5D', '#ED6F3A')
    
    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

def draw_interpolated_curve(ax, interpolator:Interpolator, points:PointList, node_count:int, label:str, line_colour:str, point_colour:str, time_limit:float|None=None):
    t_anchors = interpolator.calculate_time_anchors(points)
    t_samples = np.linspace(0, (time_limit if time_limit != None else t_anchors[-1]), node_count)
    print(f"{label}:\n    Time anchors:\n     {repr(t_anchors)}\n    Time samples:\n     {(repr(t_samples))}")  # TODO: Remove, for debugging

    # Calculate interpolated points
    interpolated_points = np.empty((node_count, 3))  # Pre-allocate 2D array
    for i, t in enumerate(t_samples):
        if t > t_anchors[-1]:  # Sample not in range / on curve
            interpolated_points = interpolated_points[:i]  # Resize
            break
        
        interpolated_points[i] = interpolator.interpolate_point(t, t_anchors, points)

    # Draw
    new_points_mask = ~np.isin(interpolated_points, points).all(axis=1)
    draw_parametric_function(ax, interpolated_points, line_colour, label)
    plot_points(ax, interpolated_points[new_points_mask], point_colour, '(Interpolated points)')

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
