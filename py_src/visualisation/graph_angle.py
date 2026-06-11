import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D, Point3DList, Vector
from data_structures import AngleLimits, ArcSettings, JointAngle
from data_structures import Standard3DUnitVectors as STD_UNIT
from kinematic_controller.fk_solver import degrees_to_radians, calculate_joint_positions, _get_unit_vectors_of_a_plane
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE


GREEN_COLOUR = "#15F015"
GREY_COLOUR  = "#7F7F7F"
RED_COLOUR   = "#F01515"


def _get_arc_points(t:npt.NDArray[np.float64], arc:ArcSettings) -> Point3DList:
    basis = np.array([arc.u_unit, arc.v_unit])
    trig = np.column_stack([np.cos(t), np.sin(t)])
    
    arc_points = (arc.pivot_point + arc.radius * (trig @ basis)).T
    return arc_points

def _draw_arc(ax, arc_points:Point3DList, colour:str) -> None:
    arc_x, arc_y, arc_z = arc_points
    ax.plot(arc_x, arc_y, arc_z, color=colour, linewidth=2.5)

def _draw_joint(ax, angle:JointAngle, colour:str, arc:ArcSettings):
    joint_min_angle, joint_max_angle = angle.limits
    current_angle = angle.end
    ref_angle     = joint_min_angle

    # Bound checks
    if current_angle < joint_min_angle:  # Under
        colour    = RED_COLOUR
    elif current_angle > joint_max_angle:  # Over
        ref_angle = joint_max_angle
        colour    = RED_COLOUR

    # Full range
    full_range_in_degrees = angle.get_total_angle_in_degrees(joint_min_angle, joint_max_angle)
    full_range_step_count = int(np.round(full_range_in_degrees))
    full_range_t_values   = np.linspace(joint_min_angle, joint_max_angle, full_range_step_count)
    full_range_points     = _get_arc_points(full_range_t_values, arc)
    _draw_arc(ax, full_range_points, GREY_COLOUR)

    # Angle
    angle_from_ref_in_degrees = angle.get_total_angle_in_degrees(ref_angle, current_angle)
    step_count   = int(np.round(angle_from_ref_in_degrees))
    t_values     = np.linspace(ref_angle, current_angle, step_count)
    angle_points = _get_arc_points(t_values, arc)
    _draw_arc(ax, angle_points, colour)

# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:npt.NDArray[np.void]):
    if len(angles) != 3:  # Parameter check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if len(joint_limits) != 3:   # Parameter check
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim3d([-0.4, 0.4])
    ax.set_ylim3d([-0.4, 0.4])
    ax.set_zlim3d([-0.4, 0.4])
    ax.set_box_aspect((1, 1, 1)) 

    # Constants
    POINT_NAMES   = ["Abductor", "Hip", "Knee", "Foot"]
    POINT_COLOURS  = ['#34495e', '#e74c3c', '#2ecc71', '#9b59b6']

    # Unpack angles
    abductor_angle, hip_angle, knee_relative_angle = angles
    knee_absolute_angle = hip_angle + knee_relative_angle

    # Get points
    abductor_pos, hip_pos, knee_pos, foot_pos = calculate_joint_positions(origin, angles)
    points = np.array([abductor_pos, hip_pos, knee_pos, foot_pos])

    # Movement Plane: Abductor-Hip vector is normal to the movement plane
    plane_u_unit, plane_v_unit = _get_unit_vectors_of_a_plane(hip_pos)


    # Linkages
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', color='#2c3e50', linewidth=4, markersize=8, label='Linkages', zorder=0)
    for i, point in enumerate(points):
        ax.scatter(point[0], point[1], point[2], color=POINT_COLOURS[i], label=POINT_NAMES[i], s=120, zorder=0)

    # Joint Angles (arcs)
    ARC_RADIUS = 0.05

    abductor_joint = JointAngle(0,         abductor_angle,      joint_limits[0])
    hip_joint      = JointAngle(0,         hip_angle,           joint_limits[1])
    knee_joint     = JointAngle(hip_angle, knee_absolute_angle, joint_limits[2])

    abductor_arc = ArcSettings(abductor_pos, ARC_RADIUS, STD_UNIT.Y,   STD_UNIT.Z)    # World YZ plane
    hip_arc      = ArcSettings(hip_pos,      ARC_RADIUS, plane_u_unit, plane_v_unit)  # Movement plane
    knee_arc     = ArcSettings(knee_pos,     ARC_RADIUS, plane_u_unit, plane_v_unit)  # Movement plane

    _draw_joint(ax, abductor_joint, GREEN_COLOUR, abductor_arc)
    _draw_joint(ax, hip_joint,      GREEN_COLOUR, hip_arc)
    _draw_joint(ax, knee_joint,     GREEN_COLOUR, knee_arc)


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
        degrees_to_radians(-90),
        degrees_to_radians(-90)
    ], dtype=np.float64)
    joint_limits = np.array([(-1.31, 2.2), (-0.5, 3.14), (0.0, 1.1)], dtype=AngleLimits)

    show_leg(origin, angles, joint_limits)