import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from data_structures import Point3D, Point3DList, Vector, Point2DList
from data_structures import AngleLimits, ArcSettings, JointAngle
from data_structures import Standard3DUnitVectors as STD_UNIT
from kinematic_controller.fk_solver import _ANGLE_ZERO_OFFSETS, degrees_to_radians, calculate_joint_positions, _get_unit_vectors_of_a_plane, _polar_to_cartesian_coordinate
from kinematic_controller.ik_solver import _HIP_ABDUCTOR_ROT_RANGE, _FRONT_HIP_ROT_RANGE, _BACK_HIP_ROT_RANGE, _KNEE_ROT_RANGE
from kinematic_controller.ik_solver import IK_Solver
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

GREEN_COLOUR = "#15F015"
GREY_COLOUR  = "#7F7F7F"
RED_COLOUR   = "#F01515"


def _point_to_str(point:Point3D, accuracy_dp:int = 3) -> str:
    slot_width:int = 3 + accuracy_dp

    def __format_number(number:float) -> str:
        _str = f"{np.round(number, accuracy_dp):<{slot_width}}"
        return _str if (number < 0) else (" " + _str[:-1])

    return f"({__format_number(point[0])}, {__format_number(point[1])}, {__format_number(point[2])})"

def _is_angle_in_range(angle:float, lower_bound:float, upper_bound:float, accuracy_dp:int = 4) -> bool:
    TAU = 2 * np.pi
    
    angle_pos  = (angle       - lower_bound) % TAU
    range_span = (upper_bound - lower_bound) % TAU
    
    in_range = np.round(angle_pos, accuracy_dp) <= np.round(range_span, accuracy_dp)
    return in_range

# Uses Shoelace formula
def _is_hole(vertices:Point2DList) -> bool:
    x, y = vertices.T
    area = 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])
    return not (area < 0)  # Not because results are backwards (werid input)

def _plot_text_on_plane(ax, text:str, font:FontProperties, origin:Point3D, u_unit:Vector, v_unit:Vector, colour='black', size=0.05, zorder=0):    
    text_path = TextPath((0, 0), text, size=size, prop=font)
    text_polygons_2d = text_path.to_polygons()

    # Project vertices onto plane in 3D world
    text_polygons_3d = []
    for polygon_2d in text_polygons_2d:
        local_x, local_y = polygon_2d.T
        
        polygon_3d = (origin +
                      local_x[:, np.newaxis] * u_unit +
                      local_y[:, np.newaxis] * v_unit)
        
        text_polygons_3d.append(polygon_3d)

    # Stitch holes together
    hole_map = np.array([_is_hole(p) for p in text_polygons_2d], dtype=bool)
    processed_text_polygons_3d = []
    for i, polygon_3d in enumerate(text_polygons_3d):
        if hole_map[i]:  # Hole
            solid = processed_text_polygons_3d[-1]
            hole  = polygon_3d
            stiched_polygon = np.vstack([solid, hole, hole[0], solid[0]])

            processed_text_polygons_3d[-1] = stiched_polygon
        else:            # Solid shape
            processed_text_polygons_3d.append(polygon_3d)

    # Render text
    text_collection = Poly3DCollection(processed_text_polygons_3d, edgecolor=None, facecolor=colour, zorder=zorder)
    ax.add_collection3d(text_collection)

def _get_arc_vertices(t:npt.NDArray[np.float64], arc:ArcSettings) -> Point3DList:
    basis = np.array([arc.u_unit, arc.v_unit])
    trig = np.column_stack([np.cos(t), np.sin(t)])
    
    outer_radius = arc.radius
    inner_radius = outer_radius - arc.width

    outer_arc_points = (arc.pivot_point + outer_radius * (trig @ basis)).T
    inner_arc_points = (arc.pivot_point + inner_radius * (trig @ basis)).T
    inner_arc_points = np.flip(inner_arc_points, axis=1)

    arc_vertices = np.concatenate((outer_arc_points, inner_arc_points), axis=1)
    return arc_vertices

def _plot_flat(ax, points:Point3DList, colour:str, zorder:int = 0) -> None:
    surface = Poly3DCollection([points.T], edgecolor=None, facecolor=colour, zorder=zorder)
    ax.add_collection3d(surface)

def _draw_arc(ax, arc_points:Point3DList, colour:str, zorder:int = 0) -> None:
    _plot_flat(ax, arc_points, colour, zorder)

def _draw_joint(ax, angle:JointAngle, colour:str, arc:ArcSettings, zero_offset:float, zorder:int = 0):
    joint_min_angle, joint_max_angle = angle.limits
    joint_min_angle += angle.start
    joint_max_angle += angle.start

    ref_angle     = joint_min_angle
    current_angle = angle.end

    range_angle_min = current_angle

    # Bound checks
    boundcheck_str = "OK"
    accuracy:int = 4
    is_under = (np.round(current_angle, accuracy) < np.round(joint_min_angle, accuracy))
    is_over  = (np.round(current_angle, accuracy) > np.round(joint_max_angle, accuracy))
    if is_under:
        colour = RED_COLOUR
        range_angle_min = ref_angle
        boundcheck_str  = "UNDER"  # TODO: Remove, for debugging
    elif is_over:
        colour = RED_COLOUR
        range_angle_min = joint_min_angle
        ref_angle       = joint_max_angle
        boundcheck_str  = "OVER"  # TODO: Remove, for debugging

    # Print joint status
    limits_str = f"(min: {np.round(np.degrees(angle.limits.minimum), 2):>7}, max: {np.round(np.degrees(angle.limits.maximum), 2):>7})"
    off_by     = angle.get_total_angle_in_degrees(ref_angle, current_angle)
    off_by_str = "(In Range)" if (boundcheck_str == "OK") else f"{np.round(off_by, 2)}"
    angle_range_str  = f"{np.round(np.degrees(angle.start), 2):>7} to {np.round(np.degrees(angle.end), 2):>7}"
    curr_angle_str   = np.round(np.degrees(current_angle), 2)
    ref_angle_str    = np.round(np.degrees(ref_angle), 2)
    actual_angle     = angle.get_total_angle_in_degrees(angle.start, angle.end)
    actual_angle_str = f"{np.round(actual_angle, 2)};"
    print(f"{boundcheck_str:<5} {off_by_str:<10} : curr: {curr_angle_str:>7} | ref: {ref_angle_str:>7} | (angle: {actual_angle_str:<8} {angle_range_str}) | {limits_str}")

    # Apply zero offset (To align visually)
    current_angle   += zero_offset
    ref_angle       += zero_offset
    joint_min_angle += zero_offset
    joint_max_angle += zero_offset
    range_angle_min += zero_offset

    # Full range
    range_angle_max = joint_max_angle
    if not np.isclose(range_angle_min, range_angle_max, 0.0001):
        full_range_in_degrees = angle.get_total_angle_in_degrees(range_angle_min, range_angle_max)
        full_range_step_count = int(np.round(full_range_in_degrees))
        full_range_t_values   = np.linspace(range_angle_min, range_angle_max, full_range_step_count)
        full_range_points     = _get_arc_vertices(full_range_t_values, arc)
        _draw_arc(ax, full_range_points, GREY_COLOUR, zorder=zorder)

    # Angle
    if not np.isclose(ref_angle, current_angle, 0.0001):
        angle_from_ref_in_degrees = angle.get_total_angle_in_degrees(ref_angle, current_angle)
        step_count   = int(np.round(angle_from_ref_in_degrees))
        t_values     = np.linspace(ref_angle, current_angle, step_count)
        angle_points = _get_arc_vertices(t_values, arc)
        _draw_arc(ax, angle_points, colour, zorder=zorder+1)

    # Text annotation
    font:FontProperties = FontProperties(family="Arial", weight="normal")
    angle_str:str = f"{np.round(np.degrees(angle.end - angle.start), 2)}°"

    text_colour = RED_COLOUR if (is_under or is_over) else 'black'
    text_size   = 0.05
    text_height = text_size
    text_width  = len(angle_str) * (text_size / 2)

    distance_from_pivot  = (arc.radius + 0.015)
    range_bisector_angle = (joint_min_angle +  joint_max_angle) / 2

    lower_overlap_angle = (range_bisector_angle - np.radians(10))
    upper_overlap_angle = (range_bisector_angle + np.radians(25))
    will_text_overlap_linkage = _is_angle_in_range(current_angle, lower_overlap_angle, upper_overlap_angle)
    if (will_text_overlap_linkage):
        range_bisector_angle = (current_angle + np.radians(25))

    horizontal_offset = text_width * np.cos(range_bisector_angle)
    vertical_offset   = text_height * np.sin(range_bisector_angle)
    if (horizontal_offset > 0):
        horizontal_offset = 0
    if (vertical_offset > 0):
        vertical_offset = 0
    
    text_pos_vector_2d   = _polar_to_cartesian_coordinate(distance_from_pivot, range_bisector_angle)
    text_pos_vector_2d  += np.array([horizontal_offset, vertical_offset])  # Offset: Align textbox
    text_origin:Point3D  = (arc.pivot_point +
                            text_pos_vector_2d[0] * arc.u_unit +
                            text_pos_vector_2d[1] * arc.v_unit)

    _plot_text_on_plane(ax, angle_str, font, text_origin, arc.u_unit, arc.v_unit, text_colour, text_size)

# TODO: Add show movement plane option
def show_leg(origin:Point3D, angles:npt.NDArray[np.float64], joint_limits:npt.NDArray[np.void], is_left_side:bool, is_front_leg:bool):
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


    # Linkages
    ax.plot(points[:, 0], points[:, 1], points[:, 2], '-o', color=LINKAGE_COLOUR, linewidth=4, markersize=8, label='Linkages', zorder=10)
    for i, point in enumerate(points):
        ax.scatter(point[0], point[1], point[2], color=POINT_COLOURS[i], label=POINT_NAMES[i], s=120, zorder=31)

    # Joint Angles (arcs)
    ARC_RADIUS = 0.05
    ARC_WIDTH  = 0.005

    abductor_joint = JointAngle(0,         abductor_angle,      joint_limits[0])
    hip_joint      = JointAngle(0,         hip_angle,           joint_limits[1])
    knee_joint     = JointAngle(hip_angle, knee_absolute_angle, joint_limits[2])

    abductor_arc = ArcSettings(abductor_pos, ARC_RADIUS, ARC_WIDTH, -STD_UNIT.X,   STD_UNIT.Z)   # Normal to the world YZ plane
    hip_arc      = ArcSettings(hip_pos,      ARC_RADIUS, ARC_WIDTH, plane_u_unit, plane_v_unit)  # Movement plane
    knee_arc     = ArcSettings(knee_pos,     ARC_RADIUS, ARC_WIDTH, plane_u_unit, plane_v_unit)  # Movement plane

    _draw_joint(ax, abductor_joint, GREEN_COLOUR, abductor_arc, _ANGLE_ZERO_OFFSETS[0], 0)
    _draw_joint(ax, hip_joint,      GREEN_COLOUR, hip_arc,      _ANGLE_ZERO_OFFSETS[1], 20)
    _draw_joint(ax, knee_joint,     GREEN_COLOUR, knee_arc,     (_ANGLE_ZERO_OFFSETS[1] + _ANGLE_ZERO_OFFSETS[2]), 20)


    # Position logs
    print(f"     Abductor pos: {_point_to_str(abductor_pos)}")
    print(f"          Hip pos: {_point_to_str(hip_pos)}")
    print(f"         Knee pos: {_point_to_str(knee_pos)}")
    print(f"     ->  Foot pos: {_point_to_str(foot_pos)}")


    ax.view_init(elev=15, azim=(50 if is_left_side else 130))  # TODO: Uncomment, testing
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('X Axis', labelpad=10)
    ax.set_ylabel('Y Axis', labelpad=10)
    ax.set_zlabel('Z Axis', labelpad=10)
    ax.legend(bbox_to_anchor=(1.325, -0.125), loc='lower right')
    plt.title(f"{'F' if is_front_leg else 'B'}{'L' if is_left_side else 'R'} leg")
    plt.show()

# IK Test
def main():
    is_left_side = False
    is_front_leg = True

    origin = np.array([0, 0.5, 0], dtype=np.float64)
    joint_limits = np.array([
        _HIP_ABDUCTOR_ROT_RANGE,
        (_FRONT_HIP_ROT_RANGE if is_front_leg else _BACK_HIP_ROT_RANGE),
        _KNEE_ROT_RANGE
    ], dtype=AngleLimits)

    target:Point3D = np.array([0.1, 0.1, 0.1], dtype=np.float64)

    ik_solver = IK_Solver()
    ik_solver._solve(origin, target)

    angles = np.array([
        degrees_to_radians(-15),
        degrees_to_radians(60),
        degrees_to_radians(-48)
    ], dtype=np.float64)

    # show_leg(origin, angles, joint_limits, is_left_side, is_front_leg)