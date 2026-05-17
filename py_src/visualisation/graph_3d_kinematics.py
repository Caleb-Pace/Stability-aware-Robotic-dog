import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from kinematic_controller.gait_definition import Gait, LEG_COUNT
from kinematic_controller.stepper import step, apply_offset

def show_full_gait(gait:Gait):
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim3d([-20, 20])
    ax.set_ylim3d([-20, 20])
    ax.set_zlim3d([0, 20])
    ax.set_box_aspect((1, 1, 1)) 

    # Mark Origin
    ax.plot([0], [0], [0], 'ro', markersize=3, label='Origin')

    # Get foot positions
    gait_steps = np.array([apply_offset(step(gait, i)) for i in range(gait.steps_in_gait)], dtype=np.float64)
    points_by_leg = np.moveaxis(gait_steps, 1, 0)
    
    # Graph settings
    colour_map = 'plasma'
    leg_labels = ["FL", "FR", "BL", "BR"]

    # Plot foot positions for each leg
    for i in range(LEG_COUNT):
        points = points_by_leg[i]
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        by_point_number = np.arange(len(x))

        _plot_line_with_gradient(ax, x, y, z, colour_map)
        ax.scatter(x, y, z, c=by_point_number, cmap=colour_map, label=leg_labels[i], s=5)

    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

def show_motor_positions(gait:Gait):
    pass

def show_ik_constraints():
    pass

def _plot_line_with_gradient(ax, x, y, z, cmap:str='viridis'):
    # Get line segments
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Create 3D line
    lc = Line3DCollection(segments, cmap=cmap, array=np.arange(len(x)), linewidth=1.5, alpha=0.7)
    ax.add_collection3d(lc)