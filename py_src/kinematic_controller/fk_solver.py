import numpy as np
import numpy.typing as npt
from data_structures import Point2D, Point3D, Vector
from kinematic_controller.ik_solver import _HIP_OFFSET, _THIGH_LENGTH, _CALF_LENGTH
from kinematic_controller.ik_solver import _ANGLE_ZERO_OFFSETS
from kinematic_controller.ik_solver import get_unit_vectors_of_a_plane


def degrees_to_radians(deg:float) -> float:
    return deg * (np.pi / 180)

def _polar_to_cartesian_coordinate(distance:float, angle:float) -> Point2D:
    return np.array([
        distance * np.cos(angle),
        distance * np.sin(angle)
    ])

def _spherical_to_cartesian_coordinate(distance_r:float, azimuth_angle:float, polar_angle:float, start_point:Point3D|None = None) -> Point3D:
    if start_point is None:
        start_point = np.array([0,0,0])

    return start_point + np.array([
        (distance_r * np.sin(polar_angle) * np.cos(azimuth_angle)),
        (distance_r * np.sin(polar_angle) * np.sin(azimuth_angle)),
        (distance_r * np.cos(polar_angle)),
    ])

def _convert_local_to_world_coordinate(local_coordinate:Point2D, plane_anchor_point:Point3D, u_unit:Vector, v_unit:Vector) -> Point3D:
    x_prime, y_prime = local_coordinate
    return plane_anchor_point + (x_prime * u_unit) + (y_prime * v_unit)

def calculate_joint_positions(origin:Point3D, angles:npt.NDArray[np.float64], is_left_side:bool = False):
    AZIMUTH_POS_Y_ANGLE = degrees_to_radians(0 if is_left_side else 180)
    
    if len(angles) != 3:  # Safety check
        raise IndexError(f"3 angles must be provided! ({len(angles)} != 3)")
    angles += _ANGLE_ZERO_OFFSETS  # Apply angle offsets
    abductor_angle, hip_angle, knee_relative_angle = angles

    # Breadth (yz) plane
    abductor_pos:Point3D = origin

    # Movement plane
    movement_plane_anchor_point:Point3D = _spherical_to_cartesian_coordinate(_HIP_OFFSET, AZIMUTH_POS_Y_ANGLE, abductor_angle, abductor_pos)
    movement_plane_normal_vector:Vector = movement_plane_anchor_point
    u_unit, v_unit                      = get_unit_vectors_of_a_plane(movement_plane_normal_vector)

    #    Hip position
    hip_pos:Point3D = movement_plane_anchor_point

    #    Knee position
    knee_local_pos:Point2D = _polar_to_cartesian_coordinate(_THIGH_LENGTH, hip_angle)
    knee_pos:Point3D       = _convert_local_to_world_coordinate(knee_local_pos, hip_pos, u_unit, v_unit)

    #    Foot (End effector) position
    knee_absolute_angle = hip_angle + knee_relative_angle
    foot_local_pos:Point2D = _polar_to_cartesian_coordinate(_CALF_LENGTH, knee_absolute_angle)
    foot_pos:Point3D       = _convert_local_to_world_coordinate(foot_local_pos, knee_pos, u_unit, v_unit)

    return abductor_pos, hip_pos, knee_pos, foot_pos
