import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D, Point3DList, Vector
from kinematic_controller.fk_solver import degrees_to_radians, calculate_joint_positions, _get_unit_vectors_of_a_plane
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE

limits_dtype = np.dtype([('min', 'f8'), ('max', 'f8')])
JointLimitsArray = npt.NDArray[np.void]

def _get_arc_points(pivot_point:Point3D, start_angle:float, end_angle:float, u_unit:Vector, v_unit:Vector, arc_radius:float) -> Point3DList:
    total_angle = np.abs(start_angle) + np.abs(end_angle)
    total_degrees = int(np.round(total_angle * (180/np.pi)))  # Convert Radians to Degrees
    t = np.linspace(start_angle, end_angle, total_degrees)

    basis = np.array([u_unit, v_unit])
    trig = np.column_stack([np.cos(t), np.sin(t)])
    arc_points = (pivot_point + arc_radius * (trig @ basis)).T
    print(arc_points.shape)
    return arc_points

def _draw_arc(ax, arc_points:Point3DList, color:str) -> None:
    arc_x, arc_y, arc_z = arc_points
    ax.plot(arc_x, arc_y, arc_z, color=color, linewidth=2.5)

# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:JointLimitsArray|None = None):
    if len(angles) != 3:  # Parameter check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if joint_limits and len(joint_limits) != 3:   # Parameter check
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim3d([-0.4, 0.4])
    ax.set_ylim3d([-0.4, 0.4])
    ax.set_zlim3d([-0.4, 0.4])
    ax.set_box_aspect((1, 1, 1)) 

    POINT_NAMES   = ["Abductor", "Hip", "Knee", "Foot"]
    POINT_COLORS  = ['#34495e', '#e74c3c', '#2ecc71', '#9b59b6']
    GREEN_COLOR = "#15F015"
    RED_COLOR   = "#F01515"

    # Unpack angles
    abductor_angle, hip_angle, knee_relative_angle = angles
    knee_absolute_angle = hip_angle + knee_relative_angle

    # Get points
    abductor_pos, hip_pos, knee_pos, foot_pos = calculate_joint_positions(origin, angles)
    points = np.array([abductor_pos, hip_pos, knee_pos, foot_pos])

    # Linkages
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', color='#2c3e50', linewidth=4, markersize=8, label='Linkages', zorder=0)
    for i, point in enumerate(points):
        ax.scatter(point[0], point[1], point[2], color=POINT_COLORS[i], label=POINT_NAMES[i], s=120, zorder=0)

    # Get unit vectors
    std_x_unit = np.array([1, 0, 0])
    std_y_unit = np.array([0, 1, 0])
    std_z_unit = np.array([0, 0, 1])
    
    #   Movement Plane: Abductor-Hip vector is normal to the movement plane
    plane_u_unit, plane_v_unit = _get_unit_vectors_of_a_plane(hip_pos)

    # Angles (arcs)
    ARC_RADIUS = 0.05
    arc_points = _get_arc_points(abductor_pos, 0, abductor_angle, std_z_unit, std_y_unit, ARC_RADIUS)
    _draw_arc(ax, arc_points, GREEN_COLOR)

    ax.view_init(elev=25, azim=65)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('X Axis', labelpad=10)
    ax.set_ylabel('Y Axis', labelpad=10)
    ax.set_zlabel('Z Axis', labelpad=10)
    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

# IK Test
def main():
    origin = np.array([0, 0, 0], dtype=np.float64)
    angles = np.array([
        degrees_to_radians(45),
        degrees_to_radians(90),
        degrees_to_radians(-90)
    ], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (3.14, -0.5), (0.0, 1.1)], dtype=limits_dtype)

    show_leg(origin, angles)