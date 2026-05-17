import numpy as np
import matplotlib.pyplot as plt
import interpolation
from interpolation import Interpolator
from data_structures import PointList

# Show plot
def show_interpolated_curves(interpolator:Interpolator, alt_interpolator:Interpolator|None = None):
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim3d([-10, 10])
    ax.set_ylim3d([-10, 10])
    ax.set_zlim3d([0, 10])
    ax.set_box_aspect((1, 1, 1)) 

    knots_base = np.array([
        [-9.0, 0.0,  0.0],
        [-4.5, 0.6,  3.0],
        [-3.0, 0.8,  3.75],
        [ 0.0, 1.0,  4.0]
    ])
    knots_set_a = np.array([
        [ 3.0, 0.8,  3.75],
        [ 4.5, 0.6,  3.0],
        [ 9.0, 0.0,  0.0]
    ])
    knots_set_b = np.array([
        [ 3.0, 2.0,  3.5],
        [ 4.0, 2.5,  2.5],
        [ 5.0, 3.0,  1.5],
        [ 7.0, 4.0,  0.0]
    ])
    knots = np.concatenate((knots_base, knots_set_a))
    knots_alt = np.concatenate((knots_base, knots_set_b))

    # Note: Linked to amount of points
    node_count = 32  # sample rate / accuracy
    time_limit = 13  # Common limit for absolute time (shares time samples)

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
    interpolated_points, _ = interpolator.compute_interpolated_points(points, node_count, time_limit)

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
