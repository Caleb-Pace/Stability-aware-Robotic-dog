import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D, Point3DList, Vector
from data_structures import AngleLimits, ArcSettings, JointAngle
from data_structures import Standard3DUnitVectors as STD_UNIT
from kinematic_controller.fk_solver import _ANGLE_ZERO_OFFSETS, degrees_to_radians, calculate_joint_positions, _get_unit_vectors_of_a_plane
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

GREEN_COLOUR = "#15F015"
GREY_COLOUR  = "#7F7F7F"
RED_COLOUR   = "#F01515"


def _get_arc_points(t:npt.NDArray[np.float64], arc:ArcSettings) -> Point3DList:
    basis = np.array([arc.u_unit, arc.v_unit])
    trig = np.column_stack([np.cos(t), np.sin(t)])
    
    arc_points = (arc.pivot_point + arc.radius * (trig @ basis)).T
    return arc_points

def _get_arc_vertices(t:npt.NDArray[np.float64], width:float, arc:ArcSettings) -> Point3DList:
    basis = np.array([arc.u_unit, arc.v_unit])
    trig = np.column_stack([np.cos(t), np.sin(t)])
    
    outer_radius = arc.radius
    inner_radius = outer_radius - width

    outer_arc_points = (arc.pivot_point + outer_radius * (trig @ basis)).T
    inner_arc_points = (arc.pivot_point + inner_radius * (trig @ basis)).T
    inner_arc_points = np.flip(inner_arc_points, axis=1)

    arc_vertices = np.concatenate((outer_arc_points, inner_arc_points), axis=1)
    print(arc_vertices.shape)
    return arc_vertices

def _plot_flat(ax, points:Point3DList, colour:str, zorder:int = 0) -> None:
    surface = Poly3DCollection([points.T], edgecolor='b', facecolor=colour, zorder=zorder)
    ax.add_collection3d(surface)

def _draw_arc(ax, arc_points:Point3DList, colour:str, zorder:int = 0) -> None:
    arc_x, arc_y, arc_z = arc_points
    _plot_flat(ax, arc_points, colour, zorder)
    # ax.plot(arc_x, arc_y, arc_z, color=colour, linewidth=2.5, zorder=zorder)

def _draw_joint(ax, angle:JointAngle, colour:str, arc:ArcSettings, zero_offset:float, zorder:int = 0):
    joint_min_angle, joint_max_angle = angle.limits
    joint_min_angle += angle.start
    joint_max_angle += angle.start

    current_angle = angle.end
    ref_angle     = joint_min_angle

    # Bound checks
    boundcheck_str = "OK"  # TODO: Remove, for debugging
    if current_angle < joint_min_angle:  # Under
        colour    = RED_COLOUR
        boundcheck_str = "UNDER"  # TODO: Remove, for debugging
    elif current_angle > joint_max_angle:  # Over
        ref_angle = joint_max_angle
        colour    = RED_COLOUR
        boundcheck_str = "OVER"  # TODO: Remove, for debugging

    # TODO: Remove, for debugging
    limits_str = f"(min: {np.round(np.degrees(angle.limits.minimum), 2):>7}, max: {np.round(np.degrees(angle.limits.maximum), 2):>7})"
    off_by     = angle.get_total_angle_in_degrees(ref_angle, current_angle)
    off_by_str = "(In Range)" if (boundcheck_str == "OK") else f"{np.round(off_by, 2)}"
    angle_str  = f"{np.round(np.degrees(angle.start), 2):>7} to {np.round(np.degrees(angle.end), 2):>7}"
    curr_angle_str   = np.round(np.degrees(current_angle), 2)
    actual_angle     = angle.get_total_angle_in_degrees(angle.start, angle.end)
    actual_angle_str = f"{np.round(actual_angle, 2)};"
    print(f"{boundcheck_str:<5} {off_by_str:<10} : curr: {curr_angle_str:>7} | (angle: {actual_angle_str:<8} {angle_str}) | {limits_str}")

    # Apply zero offset (To align visually)
    current_angle   += zero_offset
    ref_angle       += zero_offset
    joint_min_angle += zero_offset
    joint_max_angle += zero_offset

    tmp_width = 0.005  # TODO: Remove, for testing

    # Full range
    full_range_in_degrees = angle.get_total_angle_in_degrees(joint_min_angle, joint_max_angle)
    full_range_step_count = int(np.round(full_range_in_degrees))
    full_range_t_values   = np.linspace(joint_min_angle, joint_max_angle, full_range_step_count)
    full_range_points     = _get_arc_vertices(full_range_t_values, tmp_width, arc)
    # _draw_arc(ax, full_range_points, GREY_COLOUR, zorder=zorder)

    # Angle
    angle_from_ref_in_degrees = angle.get_total_angle_in_degrees(ref_angle, current_angle)
    step_count   = int(np.round(angle_from_ref_in_degrees))
    t_values     = np.linspace(ref_angle, current_angle, step_count)
    angle_points = _get_arc_vertices(t_values, tmp_width, arc)
    _draw_arc(ax, angle_points, colour, zorder=zorder+1)

    # TODO: Remove, for testing
    angle_x, angle_y, angle_z = angle_points
    ax.scatter(angle_x, angle_y, angle_z, c=np.linspace(0, 1, len(angle_x)), cmap='plasma')

# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:npt.NDArray[np.void], is_left_side:bool):
    if len(angles) != 3:  # Parameter check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    if len(joint_limits) != 3:   # Parameter check
        raise IndexError(f"3 joint limits must be provided! ({len(joint_limits)} != 3)")

    ax = plt.figure().add_subplot(projection='3d')
    # ax.set_xlim3d([-0.4, 0.4])
    # ax.set_ylim3d([-0.4, 0.4])
    # ax.set_zlim3d([-0.4, 0.4])
    ax.set_box_aspect((1, 1, 1)) 

    # Constants
    POINT_NAMES    = ["Abductor", "Hip", "Knee", "Foot"]
    POINT_COLOURS  = ['#34495e', '#e74c3c', '#2ecc71', '#9b59b6']
    LINKAGE_COLOUR = '#2c3e50'

    # Unpack angles
    abductor_angle, hip_angle, knee_relative_angle = angles
    knee_absolute_angle = hip_angle + knee_relative_angle

    # Get points
    abductor_pos, hip_pos, knee_pos, foot_pos = calculate_joint_positions(origin, angles, is_left_side)
    points = np.array([abductor_pos, hip_pos, knee_pos, foot_pos])

    # Movement Plane: Abductor-Hip vector is normal to the movement plane
    plane_u_unit, plane_v_unit = _get_unit_vectors_of_a_plane(hip_pos)


    # # Linkages
    # ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', color=LINKAGE_COLOUR, linewidth=4, markersize=8, label='Linkages', zorder=10)
    # for i, point in enumerate(points):
    #     ax.scatter(point[0], point[1], point[2], color=POINT_COLOURS[i], label=POINT_NAMES[i], s=120, zorder=31)

    # Joint Angles (arcs)
    ARC_RADIUS = 0.05

    abductor_joint = JointAngle(0,         abductor_angle,      joint_limits[0])
    hip_joint      = JointAngle(0,         hip_angle,           joint_limits[1])
    knee_joint     = JointAngle(hip_angle, knee_absolute_angle, joint_limits[2])

    abductor_arc = ArcSettings(abductor_pos, ARC_RADIUS, STD_UNIT.X,   STD_UNIT.Z)    # Normal to the world YZ plane
    hip_arc      = ArcSettings(hip_pos,      ARC_RADIUS, plane_u_unit, plane_v_unit)  # Movement plane
    knee_arc     = ArcSettings(knee_pos,     ARC_RADIUS, plane_u_unit, plane_v_unit)  # Movement plane

    # _draw_joint(ax, abductor_joint, GREEN_COLOUR, abductor_arc, _ANGLE_ZERO_OFFSETS[0], 0)
    _draw_joint(ax, hip_joint,      GREEN_COLOUR, hip_arc,      _ANGLE_ZERO_OFFSETS[1], 20)
    # _draw_joint(ax, knee_joint,     GREEN_COLOUR, knee_arc,     (_ANGLE_ZERO_OFFSETS[1] + _ANGLE_ZERO_OFFSETS[2]), 20)


    ax.view_init(elev=15, azim=(50 if is_left_side else 130))  # TODO: Uncomment, testing
    # ax.view_init(elev=0, azim=180)  # TODO: Remove, testing
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('X Axis', labelpad=10)
    ax.set_ylabel('Y Axis', labelpad=10)
    ax.set_zlabel('Z Axis', labelpad=10)
    # ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.show()

# IK Test
def main():
    is_left_side = False
    is_front_leg = True

    angles = np.array([
        degrees_to_radians(-15),
        degrees_to_radians(0),
        degrees_to_radians(-48)
    ], dtype=np.float64)

    origin = np.array([0, 0, 0], dtype=np.float64)
    joint_limits = np.array([
        _HIP_ABDUCTOR_ROT_RANGE,
        (_FRONT_HIP_ROT_RANGE if is_front_leg else _BACK_HIP_ROT_RANGE),
        _KNEE_ROT_RANGE
    ], dtype=AngleLimits)

    show_leg(origin, angles, joint_limits, is_left_side)