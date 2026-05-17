import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from kinematic_controller.gait_definition import Gait, LEG_COUNT
from kinematic_controller.stepper import step, apply_offset

def show_full_gait(gait:Gait):
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim3d([-20, 20])
    ax.set_ylim3d([-20, 20])
    ax.set_zlim3d([0, 20])
    ax.set_box_aspect((1, 1, 1)) 

    ax.plot([0], [0], [0], 'ro', markersize=5, label='Origin')

    gait_steps = np.array([apply_offset(step(gait, i)) for i in range(gait.steps_in_gait)], dtype=np.float64)
    points_by_leg = np.moveaxis(gait_steps, 1, 0)
    # print(f"{gait_steps.shape} -> {points_by_leg.shape}")  # TODO: Remove, for debugging
    
    leg_colours = ["#C1DF6E", "#67C459", "#4DA682", "#5985A6"]
    leg_labels = ["FL", "FR", "BL", "BR"]

    for i in range(LEG_COUNT):
        points = points_by_leg[i]
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        # print(repr(points))  # TODO: Remove, for debugging

        ax.plot(   x, y, z, c=leg_colours[i], label=leg_labels[i])
        ax.scatter(x, y, z, c=leg_colours[i], label=leg_labels[i])

    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

def show_motor_positions(gait:Gait):
    pass

def show_ik_constraints():
    pass